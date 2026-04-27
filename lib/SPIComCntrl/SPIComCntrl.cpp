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
    constexpr float SERVO2_HOME_DEG = 90.0f + 9.0f; // 98.5°
    constexpr float SERVO3_HOME_DEG = 90.0f - 0.5f; // 90.0°

    // gewünschte Begrenzung relativ zur Home-Lage (+/- 20°)
    constexpr float SERVO_CLAMP_DELTA_DEG = 20.0f; 

    // Ballmodell: x_ddot = (3g/5) * theta
    constexpr float G_MM_S2 = 9810.0f;                    // mm/s^2
    constexpr float BALL_ACC_PER_RAD = (3.0f * G_MM_S2) / 5.0f;

    // Zum vorsichtigen Aktivieren: zuerst kleiner als 1 testen
    constexpr float TRAJ_FF_GAIN = 0.45f;

    // Kalman / Timing
    constexpr float CAMERA_TS_S = 0.020f; // 50 Hz
    constexpr float CONTROL_TS_S = 0.002f; // 500 Hz

    // Diese Werte mit deinen MATLAB-Werten ersetzen!
    // He = lqr(Ae.', Ce.', Qe, Re).'
    constexpr float KALMAN_HE_0 = 39.097973f;
    constexpr float KALMAN_HE_1 = 714.325806f;
    constexpr float KALMAN_HE_2 = 1.000000f;
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
    m_ballPosCntrl_x.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, CONTROL_TS_S, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
    m_ballPosCntrl_y.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, CONTROL_TS_S, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);

    // Kalman observer gain from MATLAB
    Eigen::Vector3f He;
    He << KALMAN_HE_0,
        KALMAN_HE_1,
        KALMAN_HE_2;

    // Kalman prediction runs with IMU / main loop frequency: 1 kHz
    m_kalmanX.init(m_Ts, CAMERA_TS_S, G_MM_S2, He);
    m_kalmanY.init(m_Ts, CAMERA_TS_S, G_MM_S2, He);

    // Optional safety limits
    m_kalmanX.setMaxInnovationMm(80.0f);
    m_kalmanY.setMaxInnovationMm(80.0f);

    m_kalmanX.setMaxDisturbanceRad(0.15f);
    m_kalmanY.setMaxDisturbanceRad(0.15f);
    
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

    // Standard: Konstante Soll-Position
    float xd = 0.0f;
    float yd = 0.0f;

    float xd_ddot = 0.0f;
    float yd_ddot = 0.0f;

    // // Kreis Trajektorie mit 35mm Radius und 0.2 Hz Frequenz
    // // Parameter für Kreisbahn
    // float f = 0.2f;      // Hz
    // float R = 35.0f;     // mm
    // float w = 2.0f * PI * f;

    // // Berechnung der Soll-Position auf der Kreisbahn
    // float xd = R * std::cos(2.0f * PI * f * t_s);
    // float yd = R * std::sin(2.0f * PI * f * t_s);
    
    // // Berechnung der Soll-Geschwindigkeit
    // float xd_dot  = -R * w * std::sin(w * t_s);
    // float yd_dot  =  R * w * std::cos(w * t_s);

    // // Berechnung der Soll-Beschleunigung
    // float xd_ddot = -R * w * w * std::cos(w * t_s);
    // float yd_ddot = -R * w * w * std::sin(w * t_s);

    // Read IMU data
    m_ImuData = m_Imu.getImuData();
    
    // Kalman prediction runs every 1 ms
    if (m_Imu.isCalibrated()) {

        const float roll_rad  = m_ImuData.rpy.x();
        const float pitch_rad = m_ImuData.rpy.y();

        /*
        * From your MATLAB test:
        * pitch_imu_rad -> x_ball_mm
        *
        * Therefore:
        * x-axis prediction uses pitch
        * y-axis prediction uses roll
        *
        * If the direction is wrong in the test, change the sign here.
        */
        const float angle_x_rad = pitch_rad;
        const float angle_y_rad = roll_rad;

        m_kalmanX.predict(angle_x_rad);
        m_kalmanY.predict(angle_y_rad);
    }

    // Handle SPI communication: check for new data, update control, prepare reply  
    static int missing_data_counter = 0;
    bool newDataAvailable = m_SpiSlaveDMA.hasNewData();

    if (newDataAvailable) {
        m_spiData = m_SpiSlaveDMA.getSPIData();

        const float x_meas_mm = m_spiData.data[0];
        const float y_meas_mm = m_spiData.data[1];

        const bool ballWasLost =
            (missing_data_counter * m_Ts) > VISION_TIMEOUT;

        if (ballWasLost) {
            /*
            * Ball was missing for a longer time.
            * Reset Kalman states to the new camera position.
            */
            m_kalmanX.reset(x_meas_mm, 0.0f, 0.0f);
            m_kalmanY.reset(y_meas_mm, 0.0f, 0.0f);

            float current_error_x = xd - x_meas_mm;
            float current_error_y = yd - y_meas_mm;

            m_ballPosCntrl_x.reset(current_error_x);
            m_ballPosCntrl_y.reset(current_error_y);
        } else {
            /*
            * Normal camera correction.
            * Important: call update only once per new camera measurement.
            */
            m_kalmanX.update(x_meas_mm);
            m_kalmanY.update(y_meas_mm);
        }

            missing_data_counter = 0;

        } else {
            missing_data_counter++;
    }


    if (m_Imu.isCalibrated()) {
        
        if ((missing_data_counter * m_Ts) <= VISION_TIMEOUT) {
            // Kalman-Schätzungen lesen
            const float x_hat_mm = m_kalmanX.getPositionMm();
            const float y_hat_mm = m_kalmanY.getPositionMm();

            // Calculate error between desired postion and current ball position
            // float error_x = xd - m_spiData.data[0]; //input in mm
            // float error_y = yd - m_spiData.data[1]; //input in mm
            float error_x = xd - x_hat_mm;
            float error_y = yd - y_hat_mm;

            float control_output_fb_x_grad = m_ballPosCntrl_x.update(error_x);
            float control_output_fb_y_grad = m_ballPosCntrl_y.update(error_y);

            // Feedforward aus Soll-Beschleunigung
            float theta_ff_x_rad = xd_ddot / BALL_ACC_PER_RAD;
            float theta_ff_y_rad = yd_ddot / BALL_ACC_PER_RAD;

            float theta_ff_x_grad = TRAJ_FF_GAIN * (theta_ff_x_rad * 180.0f / M_PIf);
            float theta_ff_y_grad = TRAJ_FF_GAIN * (theta_ff_y_rad * 180.0f / M_PIf);

            // Gesamt-Stellgröße
            float control_output_x_grad = control_output_fb_x_grad + theta_ff_x_grad;
            float control_output_y_grad = control_output_fb_y_grad + theta_ff_y_grad;

            // rotate control outputs
            float rotated_output_x = control_output_x_grad * cos_theta_rotation - control_output_y_grad * sin_theta_rotation;
            float rotated_output_y = control_output_x_grad * sin_theta_rotation + control_output_y_grad * cos_theta_rotation;

            // Inputs für Inverse Kinematik berechnen (Roll, Pitch, Höhe)
            m_ikInput.pitch = DegreeToRad(rotated_output_x);
            m_ikInput.roll  = -DegreeToRad(rotated_output_y);
            m_ikInput.h     = 110.5f;

            InverseKinematics3Leg::Result ikResult = m_ik.compute(m_ikInput);

            // WICHTIG: IK-Ergebnis prüfen, bevor alphaDeg verwendet wird
            if (ikResult.success) {

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
                // Falls IK fehlschlägt: sicher auf Home-Lage zurück
                m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG);
                m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG);
                m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG);
            }

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

        m_SerialStream.write(m_kalmanX.getPositionMm());     // 13 x_hat
        m_SerialStream.write(m_kalmanX.getVelocityMmS());    // 14 vx_hat
        m_SerialStream.write(m_kalmanX.getDisturbanceRad()); // 15 dx_hat

        m_SerialStream.write(m_kalmanY.getPositionMm());     // 16 y_hat
        m_SerialStream.write(m_kalmanY.getVelocityMmS());    // 17 vy_hat
        m_SerialStream.write(m_kalmanY.getDisturbanceRad()); // 18 dy_hat

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