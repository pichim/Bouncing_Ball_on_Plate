#include "SPIComCntrl.h"
#include <cmath>

// Servo & Inverse Kinematics mapping constants
namespace
{
    constexpr float IK_HOME_DEG = 90.0f;    // aus IK: roll=0, pitch=0, h=98.1
    constexpr float SERVO_MAX_DEG = 122.7f; // real nutzbarer Servobereich
    constexpr float SERVO_MIN_DEG = 0.0f;

    // Reale HOME-Winkel der 3 Servos bei waagerechter Platte
    constexpr float SERVO1_HOME_DEG = 90.0f + 6.5f; // 96.5°
    constexpr float SERVO2_HOME_DEG = 90.0f + 8.5f; // 98.5°
    constexpr float SERVO3_HOME_DEG = 90.0f + 0.0f; // 90.0°

    // gewünschte Begrenzung relativ zur Home-Lage (+/- 20°)
    constexpr float SERVO_CLAMP_DELTA_DEG = 20.0f; 
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

void SPIComCntrl::executeTask()
{
    // Return early if SPI not ready
    if (!m_spi_ready) {
        return;
    }

    // Measure delta time
    const microseconds time_us = m_Timer.elapsed_time();
    const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
    m_time_previous_us = time_us;

    // Trajektorie einmal pro Zyklus berechnen
    float t_s = duration_cast<microseconds>(time_us).count() * 1.0e-6f;

    // Konstante Soll-Position
    float xd = 0.0f;
    float yd = 0.0f;

    // // Kreis Trajektorie mit 50mm Radius und 0.1 Hz Frequenz
    // float f = 0.1f;      // Hz
    // float R = 30.0f;     // mm
    // float xd = R * std::cos(2.0f * PI * f * t_s);
    // float yd = R * std::sin(2.0f * PI * f * t_s);

    // Read IMU data
    m_ImuData = m_Imu.getImuData();
    // if (!m_Imu.isCalibrated())
    //     return;

    // Handle SPI communication: check for new data, update control, prepare reply  
    static int missing_data_counter = 0;
    bool newDataAvailable = m_SpiSlaveDMA.hasNewData();

    if (newDataAvailable) {
        m_spiData = m_SpiSlaveDMA.getSPIData();
        
    //     // Wenn der Ball länger weg war, Regler-Historie löschen, um Schock zu vermeiden
    //     if ((missing_data_counter * m_Ts) > VISION_TIMEOUT) { 
    //         m_ballPosCntrl_x.reset(0.0f); 
    //         m_ballPosCntrl_y.reset(0.0f); 
    //     }
    //     missing_data_counter = 0; // Ball ist wieder da, Counter resetten


        // NEU: Wenn der Ball gerade eben erst wieder aufgetaucht ist (nach mindestens 1 Frame Pause)
        if ((missing_data_counter * m_Ts) > VISION_TIMEOUT) { 
            // Wir setzen den Regler auf den AKTUELLEN Fehlerwert, 
            // damit delta_error im nächsten Schritt 0 ist.

            // Aktueller Fehler als Startwert für den Regler
            float current_error_x = xd - m_spiData.data[0];
            float current_error_y = yd - m_spiData.data[1];

            m_ballPosCntrl_x.reset(current_error_x); 
            m_ballPosCntrl_y.reset(current_error_y); 
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
        
        if ((missing_data_counter * m_Ts) <= VISION_TIMEOUT) {
            
            // Calculate error between desired postion and current ball position
            float error_x = xd - m_spiData.data[0]; //input in mm
            float error_y = yd - m_spiData.data[1]; //input in mm

            // Update Control output (PID) to get servo commands
            float control_output_x_grad = m_ballPosCntrl_x.update(error_x);
            float control_output_y_grad = m_ballPosCntrl_y.update(error_y);

            // rotate control outputs
            float rotated_output_x = control_output_x_grad * cos_theta_rotation - control_output_y_grad * sin_theta_rotation;
            float rotated_output_y = control_output_x_grad * sin_theta_rotation + control_output_y_grad * cos_theta_rotation;

            // Inputs für Inverse Kinematik berechnen (Roll, Pitch, Höhe)
            ikInput.pitch = -DegreeToRad(rotated_output_x);
            ikInput.roll = DegreeToRad(rotated_output_y);
            ikInput.h = 98.1f;
            
            // Inverse Kinematik berechnen
            InverseKinematics3Leg::Result ikResult = ik.compute(ikInput);
            
            // Servo commands in Grad berechnen
            float servo1_cmd_deg = SERVO1_HOME_DEG + (ikResult.alphaDeg[0] - IK_HOME_DEG);
            float servo2_cmd_deg = SERVO2_HOME_DEG + (ikResult.alphaDeg[1] - IK_HOME_DEG);
            float servo3_cmd_deg = SERVO3_HOME_DEG + (ikResult.alphaDeg[2] - IK_HOME_DEG);

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
    m_reply_data[0] = m_spiData.data[0]; // Echo servo D0 command
    m_reply_data[1] = m_spiData.data[1]; // error distance
    m_reply_data[2] = m_servo_commands[2]; // Echo servo D2 command
    m_reply_data[3] = m_ImuData.gyro.x();  // Gyro X in rad/sec
    m_reply_data[4] = m_ImuData.gyro.y();  // Gyro Y in rad/sec
    m_reply_data[5] = m_ImuData.gyro.z();  // Gyro Z in rad/sec
    m_reply_data[6] = m_ImuData.acc.x();   // Acc X in m/sec^2
    m_reply_data[7] = m_ImuData.acc.y();   // Acc Y in m/sec^2
    m_reply_data[8] = m_ImuData.acc.z();   // Acc Z in m/sec^2
    m_SpiSlaveDMA.setReplyData(m_reply_data, 9);

    // Send data over serial stream
    if (m_SerialStream.startByteReceived()) {
        m_SerialStream.write(dtime_us);            //  0 Delta time in us
        m_SerialStream.write(m_servo_commands[0]); //  1 Echo servo D0 command
        m_SerialStream.write(m_servo_commands[1]); //  2 Echo servo D1 command
        m_SerialStream.write(m_servo_commands[2]); //  3 Echo servo D2 command
        m_SerialStream.write(m_ImuData.gyro.x());  //  4 Gyro X in rad/sec
        m_SerialStream.write(m_ImuData.gyro.y());  //  5 Gyro Y in rad/sec
        m_SerialStream.write(m_ImuData.gyro.z());  //  6 Gyro Z in rad/sec
        m_SerialStream.write(m_ImuData.acc.x());   //  7 Acc X in m/sec^2
        m_SerialStream.write(m_ImuData.acc.y());   //  8 Acc Y in m/sec^2
        m_SerialStream.write(m_ImuData.acc.z());   //  9 Acc Z in m/sec^2
        m_SerialStream.write(m_ImuData.rpy.x());   // 10 Roll in rad
        m_SerialStream.write(m_ImuData.rpy.y());   // 11 Pitch in rad
        m_SerialStream.write(m_ImuData.rpy.z());   // 12 Yaw in rad
        m_SerialStream.send();
    }
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