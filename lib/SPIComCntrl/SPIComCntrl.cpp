#include "SPIComCntrl.h"


// Servo & Inverse Kinematics mapping constants
namespace
{
    constexpr float IK_HOME_DEG = 90.0f;    // aus IK: roll=0, pitch=0, h=98.1
    constexpr float SERVO_MAX_DEG = 122.7f; // real nutzbarer Servobereich
    constexpr float SERVO_MIN_DEG = 0.0f;

    // Reale HOME-Winkel der 3 Servos bei waagerechter Platte
    constexpr float SERVO1_HOME_DEG = 90.0f + 6.5f; // 96.5°
    constexpr float SERVO2_HOME_DEG = 90.0f + 9.0f; // 98.5°
    constexpr float SERVO3_HOME_DEG = 90.0f - 0.5f; // 90.0°

    // gewünschte Begrenzung relativ zur Home-Lage (+/- 20°)
    constexpr float SERVO_CLAMP_DELTA_DEG = 40.0f; 
}

SPIComCntrl::SPIComCntrl()
    : RealTimeThread(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US,
                     BBOP_SPI_COM_CNTRL_THREAD_PRIORITY,
                     BBOP_SPI_COM_CNTRL_THREAD_STACK_SIZE)
    , m_SpiSlaveDMA(BBOP_SPI_SLAVE_DMA_MOSI_PIN,
                    BBOP_SPI_SLAVE_DMA_MISO_PIN,
                    BBOP_SPI_SLAVE_DMA_SCK_PIN,
                    BBOP_SPI_SLAVE_DMA_NSS_PIN,
                    BBOP_SPI_SLAVE_DMA_THREAD_PRIORITY,
                    BBOP_SPI_SLAVE_DMA_THREAD_STACK_SIZE)
    , m_Imu(BBOP_IMU_SDA_PIN, BBOP_IMU_SCL_PIN)
    , m_servoD0(BBOP_SERVO_D0_PIN, BBOP_SERVO_PWM_PERIOD_US)
    , m_servoD1(BBOP_SERVO_D1_PIN, BBOP_SERVO_PWM_PERIOD_US)
    , m_servoD2(BBOP_SERVO_D2_PIN, BBOP_SERVO_PWM_PERIOD_US)
    , m_SerialStream(BBOP_LOG_COM_UART_TX_PIN, BBOP_LOG_COM_UART_RX_PIN)
    , m_Ts(static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f)
    , user_button(BBOP_USER_BUTTON, PullUp)
{
    // Start SPI communication; guard failure
    if (!m_SpiSlaveDMA.start()) {
        printf("SPI start() failed — check wiring, pin mapping, or DMA state.\n");
        return;
    }

    m_spi_ready = true;
    printf("SPI Communication started. Waiting for master...\n");

    //create PID-T1 controller
    // m_ballPosCntrl = PIDCntrl(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_D_S, m_Ts, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
    // m_ballPosCntrl_x.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_D_S, BALL_CTRL_TAU_R_O, m_Ts, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
    // m_ballPosCntrl_y.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_D_S, BALL_CTRL_TAU_R_O, m_Ts, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
    m_ballPosCntrl_x.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, 0.02f, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
    m_ballPosCntrl_y.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, 0.02f, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);

    // Calibrate and enable servos (normalised pulse widths)
    m_servoD0.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD1.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD2.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);

    if (!m_servoD0.isEnabled()) {
        m_servoD0.enable(DegreeToPWM(SERVO1_HOME_DEG));
    }
    if (!m_servoD1.isEnabled()) {
        m_servoD1.enable(DegreeToPWM(SERVO2_HOME_DEG));
    }
    if (!m_servoD2.isEnabled()) {
        m_servoD2.enable(DegreeToPWM(SERVO3_HOME_DEG));
    }

    // Verdrehungswinkel definieren Kamera zu Base
    camera_offset_angle_deg = 90.0f + 14.73f;
    camera_offset_angle_rad = DegreeToRad(camera_offset_angle_deg);

    // Berechne Sinus und Kosinus
    cos_theta_rotation = std::cos(camera_offset_angle_rad);
    sin_theta_rotation = std::sin(camera_offset_angle_rad);

    m_Timer.start();

    // NOTE: RealTimeThread::enable() must be called by the user after construction is complete
}

SPIComCntrl::~SPIComCntrl() = default;

InverseKinematics3Leg ik;
InverseKinematics3Leg::Input ikInput;

// Temporary variables for setpoint sequence testing
static int cntr = 0;
static int step_index = 0;
static float current_setpoint_x = 0.0f;
static float filtered_setpoint_x = 0.0f;
static float current_setpoint_y = 0.0f;
static float filtered_setpoint_y = 0.0f;

void SPIComCntrl::executeTask()
{
    // Return early if SPI not ready
    if (!m_spi_ready) {
        return;
    }

    user_button.rise(callback(this, &SPIComCntrl::toggleExecuteMainFcn));

    // Measure delta time
    const microseconds time_us = m_Timer.elapsed_time();
    const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
    m_time_previous_us = time_us;

    // Trajektorie einmal pro Zyklus berechnen
    float t_s = duration_cast<microseconds>(time_us).count() * 1.0e-6f;

    // Konstante Soll-Position
    float xd = 0.0f;
    float yd = 0.0f;
    
    // Read IMU data
    m_ImuData = m_Imu.getImuData();
    // if (!m_Imu.isCalibrated())
    //     return;

    // Handle SPI communication: check for new data, update control, prepare reply  
    static int missing_data_counter = 0;
    bool newDataAvailable = m_SpiSlaveDMA.hasNewData();

    // temporary test code for setpoint sequence

    const float setpoint_sequence[] = {0.0f, 40.0f, 0.0f, -40.0f}; // Beispiel-Sequenz mit 4 Schritten
    const int NUM_STEPS = 4;

    if (newDataAvailable) {
        m_spiData = m_SpiSlaveDMA.getSPIData();
        

    //temporary code for setpoint sequence
        cntr++;

        // Prüfen, ob die Zeit für den nächsten Schritt reif ist
        if (cntr > 500) {  // alle 20 ms ++
            cntr = 0; // Zähler zurücksetzen
            
            // Index für das Array erhöhen
            step_index++;
            
            // Wenn wir am Ende des Arrays sind, wieder bei 0 anfangen
            if (step_index >= NUM_STEPS) {
                step_index = 0;
            }
        }

        // Aktuellen Sollwert aus dem Array lesen
        current_setpoint_x = setpoint_sequence[step_index];
        current_setpoint_y = setpoint_sequence[step_index];
        //finished temporary code for setpoint sequence
        // m_reply_data[0] = current_setpoint; // Echo current setpoint in reply for logging




        if ((missing_data_counter * m_Ts) > VISION_TIMEOUT) { 
            // Wir setzen den Regler auf den AKTUELLEN Fehlerwert, 
            // damit delta_error im nächsten Schritt 0 ist.

            // Aktueller Fehler als Startwert für den Regler
            // float current_error_x = current_setpoint - m_spiData.data[0];
            // float current_error_y = current_setpoint - m_spiData.data[1];

            m_ballPosCntrl_x.reset(0.0f); 
            m_ballPosCntrl_y.reset(0.0f); 

            filtered_setpoint_x = 0.0f; // to do: reset after ball reapears + grace period
            filtered_setpoint_y = 0.0f;
        }
        
        missing_data_counter = 0;

        // // When logging via SerialStream you have to uncomment this print
        // printf("Message: %lu | Delta Time: %lu us | "
        //        "Received: [%.2f, %.2f, %.2f] | "
        //        "Header: 0x%02X | Failed: %lu | "
        //        "Readout Time: %lu us\n",
        //        m_spiData.message_count,
        //        m_spiData.last_delta_time_us,
        //        m_spiData.data[0],
        //        m_spiData.data[1],
        //        m_spiData.data[2],
        //        SPI_HEADER_SLAVE,
        //        m_spiData.failed_count,
        //        m_spiData.readout_time_us);

    } else {
        missing_data_counter++; // Kein Ball in diesem Frame
    }


    if (m_Imu.isCalibrated()) {
        
        if (((missing_data_counter * m_Ts) <= VISION_TIMEOUT)) {

            //sollwert tiefpass
            static constexpr float SETPOINT_TAU = 0.1f * BALL_CTRL_TAU_V; // Tf aus MATLAB (Ca. 1 / w_d)

            // Weichzeichnen des Sollwerts
            float alpha_sp = m_Ts / (SETPOINT_TAU + m_Ts);
            filtered_setpoint_x = filtered_setpoint_x + alpha_sp * (current_setpoint_x - filtered_setpoint_x);
            filtered_setpoint_y = filtered_setpoint_y + alpha_sp * (current_setpoint_y - filtered_setpoint_y);
            
            // Calculate error between desired postion and current ball position
            float error_x = filtered_setpoint_x - m_spiData.data[0]; //input in mm
            float error_y = filtered_setpoint_y - m_spiData.data[1]; //input in mm

            // Update Control output (PID) to get servo commands
            float control_output_x_grad = m_ballPosCntrl_x.update(error_x);
            float control_output_y_grad = m_ballPosCntrl_y.update(error_y);

            // rotate control outputs
            float rotated_output_x = control_output_x_grad * cos_theta_rotation - control_output_y_grad * sin_theta_rotation;
            float rotated_output_y = control_output_x_grad * sin_theta_rotation + control_output_y_grad * cos_theta_rotation;
            

            // Inputs für Inverse Kinematik berechnen (Roll, Pitch, Höhe)
            ikInput.pitch = DegreeToRad(rotated_output_x);
            ikInput.roll = -DegreeToRad(rotated_output_y);
            m_reply_data[6] = -DegreeToRad(rotated_output_x);
            m_reply_data[7] = DegreeToRad(rotated_output_y);
            ikInput.h = 110.5f; // original height for middle pos

            // ikInput.pitch = 0.0f;
            // ikInput.roll = 0.0f;
            // m_reply_data[0] = -DegreeToRad(rotated_output_x);
            // m_reply_data[1] = DegreeToRad(rotated_output_y);
            // ikInput.h = 110.5f - 0.0f; // original height for middle pos

            // Hoehenkompensation
            float ball_x_phys = m_spiData.data[0] * cos_theta_rotation - m_spiData.data[1] * sin_theta_rotation;
            float ball_y_phys = m_spiData.data[0] * sin_theta_rotation + m_spiData.data[1] * cos_theta_rotation;
            float compensation = ball_x_phys * tan(ikInput.pitch) - ball_y_phys * tan(ikInput.roll);
            ikInput.h = 110.5f + compensation * 1.0f;
            // ikInput.h = clamp(110.5f - m_spiData.data[2] * 1.0f, 110.5f - 30.0f, 110.5f + 30.0f); // clamp height to avoid singularities

            // Inverse Kinematik berechnen
            InverseKinematics3Leg::Result ikResult = ik.compute(ikInput);
            
            // Servo commands in Grad berechnen
            float servo1_cmd_deg = SERVO1_HOME_DEG - (ikResult.alphaDeg[0] - IK_HOME_DEG);
            float servo2_cmd_deg = SERVO2_HOME_DEG - (ikResult.alphaDeg[1] - IK_HOME_DEG);
            float servo3_cmd_deg = SERVO3_HOME_DEG - (ikResult.alphaDeg[2] - IK_HOME_DEG);

            // Zuerst auf +/-20° um die jeweilige Home-Lage clampen
            servo1_cmd_deg = clamp(servo1_cmd_deg,
                                   SERVO1_HOME_DEG - SERVO_CLAMP_DELTA_DEG,
                                   SERVO1_HOME_DEG + SERVO_CLAMP_DELTA_DEG);

            servo2_cmd_deg = clamp(servo2_cmd_deg,
                                   SERVO2_HOME_DEG - SERVO_CLAMP_DELTA_DEG,
                                   SERVO2_HOME_DEG + SERVO_CLAMP_DELTA_DEG);

            servo3_cmd_deg = clamp(servo3_cmd_deg,
                                   SERVO3_HOME_DEG - SERVO_CLAMP_DELTA_DEG,
                                   SERVO3_HOME_DEG + SERVO_CLAMP_DELTA_DEG);

            // Zusätzlicher harter Sicherheitsclamp auf den realen Servobereich
            servo1_cmd_deg = clamp(servo1_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
            servo2_cmd_deg = clamp(servo2_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
            servo3_cmd_deg = clamp(servo3_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);

            // Erst ganz am Schluss in normierten Servo-Befehl umrechnen
            m_servo_commands[0] = DegreeToPWM(servo1_cmd_deg);
            m_servo_commands[1] = DegreeToPWM(servo2_cmd_deg);
            m_servo_commands[2] = DegreeToPWM(servo3_cmd_deg);

        } else {

            // Ball weg, Servos in Mittelstellung halten
            m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG);
            m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG);
            m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG);

        }

        // Servo ansteuern
        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);

    }

    // Prepare next reply
    m_reply_data[0] = m_spiData.data[0]; // X
    m_reply_data[1] = m_spiData.data[1]; // Y
    m_reply_data[2] = filtered_setpoint_x; // Echo servo D2 command
    m_reply_data[3] = filtered_setpoint_y;  // Gyro X in rad/sec
    m_reply_data[4] = m_ImuData.gyro.y();  // Gyro Y in rad/sec
    m_reply_data[5] = m_ImuData.gyro.z();  // Gyro Z in rad/sec
    m_reply_data[6] = m_ImuData.acc.x();   // Acc X in m/sec^2
    m_reply_data[7] = m_ImuData.acc.y();   // Acc Y in m/sec^2
    m_reply_data[8] = m_ImuData.acc.z();   // Acc Z in m/sec^2
    m_SpiSlaveDMA.setReplyData(m_reply_data, 9);

    // Send data over serial stream
    if (m_SerialStream.startByteReceived()) {
        m_SerialStream.write(dtime_us);            
        m_SerialStream.write(m_spiData.data[0]); // x
        m_SerialStream.write(m_spiData.data[1]);// y
        m_SerialStream.write(m_spiData.data[2]);   // z
        
        m_SerialStream.write(filtered_setpoint_x); // 4: pitch in grad
        m_SerialStream.write(filtered_setpoint_y); // 5: roll in grad
        
        m_SerialStream.write(m_reply_data[6]); // 6: Angewendeter Pitch (Grad)
        m_SerialStream.write(m_reply_data[7]);  // 7: Angewendeter Roll (Grad)
        m_SerialStream.write(m_ImuData.acc.y());   //  8 Acc Y in m/sec^2
        m_SerialStream.write(m_ImuData.acc.z());   //  9 Acc Z in m/sec^2
        m_SerialStream.write(m_ImuData.rpy.x());   // 10 Roll in rad
        m_SerialStream.write(m_ImuData.rpy.y());   // 11 Pitch in rad
        m_SerialStream.write(m_ImuData.rpy.z());   // 12 Yaw in rad
        m_SerialStream.send();
    }

    printf(m_executeMain ? "Main task enabled\n" : "Main task disabled\n");


    if (m_executeMain) {
        if (!m_servoD0.isEnabled()) {
            m_servoD0.enable(DegreeToPWM(SERVO1_HOME_DEG));
        }
        if (!m_servoD1.isEnabled()) {
            m_servoD1.enable(DegreeToPWM(SERVO2_HOME_DEG));
        }
        if (!m_servoD2.isEnabled()) {
            m_servoD2.enable(DegreeToPWM(SERVO3_HOME_DEG));
        }
    } else {
        m_servoD0.disable();
        m_servoD1.disable();
        m_servoD2.disable();
    }
}

void SPIComCntrl::toggleExecuteMainFcn()
{
    // Toggle some execution flag
    m_executeMain = !m_executeMain;
}

float SPIComCntrl::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

float SPIComCntrl::DegreeToPWM(float degree)
{
    float pulse_width = (degree / SERVO_MAX_DEG);
    return pulse_width;
}

float SPIComCntrl::PWMToDegree(float pulse_width)
{
    float degree = pulse_width * SERVO_MAX_DEG;
    return degree;
}

float SPIComCntrl::DegreeToRad(float degree)
{
    float rad = degree * (M_PIf / 180.0f);
    return rad;
}
