#include "SPIComCntrl.h"
#include <cmath>

// Servo & Inverse Kinematics mapping constants
namespace
{
    constexpr float IK_HOME_DEG = 55.0f;    // aus IK: roll=0, pitch=0, h=98.1
    constexpr float SERVO_MAX_DEG = 122.7f; // real nutzbarer Servobereich
    constexpr float SERVO_MIN_DEG = 0.0f;

    // Reale HOME-Winkel der 3 Servos bei waagerechter Platte
    constexpr float SERVO1_HOME_DEG = IK_HOME_DEG + 0.5f;
    constexpr float SERVO2_HOME_DEG = IK_HOME_DEG + 2.8f;
    constexpr float SERVO3_HOME_DEG = IK_HOME_DEG - 2.1f;
 

    // gewünschte Begrenzung relativ zur Home-Lage (+/- 20°)
    constexpr float SERVO_CLAMP_DELTA_DEG = 20.0f; 

    // Ballmodell: x_ddot = (3g/5) * theta
    constexpr float G_MM_S2 = 9810.0f;                    // mm/s^2
    constexpr float BALL_ACC_PER_RAD = (3.0f * G_MM_S2) / 5.0f;

    // Zum vorsichtigen Aktivieren: zuerst kleiner als 1 testen
    constexpr float TRAJ_FF_GAIN = 0.45f;

    // Kalman / Timing
    constexpr float CAMERA_TS_S = 0.020f; // 50 Hz
    constexpr float CONTROL_TS_S = BBOP_SERVO_PWM_PERIOD_US * 1.0e-6f; // 333 Hz

    // Aus MATLAB:
    // He = lqr(Ae.', Ce.', Qe, Re).'
    // Kontinuierlicher Beobachtergewinn He
    constexpr float KALMAN_HE_0 = 39.097973f;
    constexpr float KALMAN_HE_1 = 714.325806f;
    constexpr float KALMAN_HE_2 = 1.000000f;

    // Variabeln für 333 hz loop
    int control_loop_counter = 0;
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
    m_ballPosCntrl_x.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, CONTROL_TS_S, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
    m_ballPosCntrl_y.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, CONTROL_TS_S, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);

    // Kalman observer gain from MATLAB
    Eigen::Vector3f He;
    He << KALMAN_HE_0,
        KALMAN_HE_1,
        KALMAN_HE_2;

    // Kalman prediction runs with control loop frequency: 1000 Hz
    m_kalmanX.init(m_Ts, CAMERA_TS_S, G_MM_S2, He);
    m_kalmanY.init(m_Ts, CAMERA_TS_S, G_MM_S2, He);

    // Optional safety limits
    m_kalmanX.setMaxInnovationMm(100.0f);
    m_kalmanY.setMaxInnovationMm(100.0f);

    m_kalmanX.setMaxDisturbanceRad(0.15f);
    m_kalmanY.setMaxDisturbanceRad(0.15f);

    // Trajectory
    m_trajectory.setHold(0.0f, 0.0f);
    // m_trajectory.setCircle(35.0f, 0.2f);
    
    // Calibrate and enable servos (normalised pulse widths)
    m_servoD0.calibratePulseMinMax(SERVO1_PULSE_MIN, SERVO1_PULSE_MAX);
    m_servoD1.calibratePulseMinMax(SERVO2_PULSE_MIN, SERVO2_PULSE_MAX);
    m_servoD2.calibratePulseMinMax(SERVO3_PULSE_MIN, SERVO3_PULSE_MAX);

    if (!m_servoD0.isEnabled()) {
        m_servoD0.enable(DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad));
    }
    if (!m_servoD1.isEnabled()) {
        m_servoD1.enable(DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad));
    }
    if (!m_servoD2.isEnabled()) {
        m_servoD2.enable(DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad));
    }

    // Verdrehungswinkel definieren Kamera zu Base
    // camera_offset_angle_deg = 90.0f + 14.73f;
    // camera_offset_angle_rad = DegreeToRad(camera_offset_angle_deg);

    // Berechne Sinus und Kosinus
    // cos_theta_rotation = std::cos(camera_offset_angle_rad);
    // sin_theta_rotation = std::sin(camera_offset_angle_rad);

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

    user_button.rise(callback(this, &SPIComCntrl::toggleExecuteMainFcn));

    // Measure delta time
    const microseconds time_us = m_Timer.elapsed_time();
    const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
    m_time_previous_us = time_us;


    // Trajektorie einmal pro Zyklus berechnen
    float t_s = duration_cast<microseconds>(time_us).count() * 1.0e-6f;

    // Standard: Konstante Soll-Position
    TrajectoryRef traj = m_trajectory.update(t_s);

    const float xd = traj.x_mm;
    const float yd = traj.y_mm;

    const float xd_dot = traj.vx_mm_s;
    const float yd_dot = traj.vy_mm_s;

    const float xd_ddot = traj.ax_mm_s2;
    const float yd_ddot = traj.ay_mm_s2;

    // Read IMU data
    m_ImuData = m_Imu.getImuData();
    
    // Kalman prediction runs every 1 ms (1 kHz)
    if (m_Imu.isCalibrated() && m_kalmanHasFirstMeasurement) {

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

        if (!m_kalmanHasFirstMeasurement)  {
            /*
            * First valid camera measurement.
            * Initialize Kalman directly at the measured ball position..
            */
            m_kalmanX.reset(x_meas_mm, 0.0f, 0.0f);
            m_kalmanY.reset(y_meas_mm, 0.0f, 0.0f);

            m_kalmanHasFirstMeasurement = true;

            float current_error_x = xd - x_meas_mm;
            float current_error_y = yd - y_meas_mm;

            // m_ballPosCntrl_x.reset(current_error_x);
            // m_ballPosCntrl_y.reset(current_error_y);
            m_ballPosCntrl_x.reset(0.0f);
            m_ballPosCntrl_y.reset(0.0f);
        } else if (ballWasLost) {
                /*
                 * Ball was missing for a longer time.  
                 * Reset Kalman states to the new camera position.
                 */
                m_kalmanX.reset(x_meas_mm, 0.0f, 0.0f);
                m_kalmanY.reset(y_meas_mm, 0.0f, 0.0f);

                const float current_error_x = xd - x_meas_mm;
                const float current_error_y = yd - y_meas_mm;

                // m_ballPosCntrl_x.reset(current_error_x);
                // m_ballPosCntrl_y.reset(current_error_y);
                m_ballPosCntrl_x.reset(0.0f);
                m_ballPosCntrl_y.reset(0.0f);
        }
        else {
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

    float error_x = 0.0f;
    float error_y = 0.0f;
    float error_vx = 0.0f;
    float error_vy = 0.0f;

    float control_output_fb_x_grad = 0.0f;
    float control_output_fb_y_grad = 0.0f;

    float theta_ff_x_rad = 0.0f;
    float theta_ff_y_rad = 0.0f;

    float theta_ff_x_grad = 0.0f;
    float theta_ff_y_grad = 0.0f;

    if (m_Imu.isCalibrated()) {
        
        if (m_kalmanHasFirstMeasurement && (missing_data_counter * m_Ts) <= VISION_TIMEOUT && m_executeMain) {
            
            // Kalman-Schätzungen lesen
            const float x_hat_mm = m_kalmanX.getPositionMm();
            const float y_hat_mm = m_kalmanY.getPositionMm();
            const float vx_hat_mm_s = m_kalmanX.getVelocityMmS();
            const float vy_hat_mm_s = m_kalmanY.getVelocityMmS();
            
            constexpr float BALL_CTRL_KV = 0.03f; // [deg / (mm/s)] vorsichtig starten



            // Feedforward aus Soll-Beschleunigung
            theta_ff_x_rad = xd_ddot / BALL_ACC_PER_RAD;
            theta_ff_y_rad = yd_ddot / BALL_ACC_PER_RAD;

            theta_ff_x_grad = TRAJ_FF_GAIN * (theta_ff_x_rad * 180.0f / M_PIf);
            theta_ff_y_grad = TRAJ_FF_GAIN * (theta_ff_y_rad * 180.0f / M_PIf);


            // herauslöschen sobald klar ist das rotation effektiv nicht mehr gebraucht wird.
            // rotate control outputs
            // float rotated_output_x = control_output_x_grad * cos_theta_rotation - control_output_y_grad * sin_theta_rotation;
            // float rotated_output_y = control_output_x_grad * sin_theta_rotation + control_output_y_grad * cos_theta_rotation;

            // countervariabeln für 333 hz loop
            control_loop_counter++;

            if (control_loop_counter >= 3) {
                control_loop_counter = 0;
                // Calculate error between desired postion and current ball position
                // Calculate error without Kalman
                // error_x = xd - m_spiData.data[0]; //input in mm
                // error_y = yd - m_spiData.data[1]; //input in mm
                // Calculate error with Kalman
                error_x = xd - x_hat_mm;
                error_y = yd - y_hat_mm;
                error_vx = xd_dot - vx_hat_mm_s;
                error_vy = yd_dot - vy_hat_mm_s;

                // ohne Kalman filter
                // float control_output_x_grad = m_ballPosCntrl_x.update(error_x);
                // float control_output_y_grad = m_ballPosCntrl_y.update(error_y);

                //mit Klaman filter
                control_output_fb_x_grad = m_ballPosCntrl_x.update(error_x) + BALL_CTRL_KV * error_vx;
                control_output_fb_y_grad = m_ballPosCntrl_y.update(error_y) + BALL_CTRL_KV * error_vy;

                // Gesamt-Stellgröße
                float control_output_x_grad = control_output_fb_x_grad + theta_ff_x_grad;
                float control_output_y_grad = control_output_fb_y_grad + theta_ff_y_grad;

                // Inputs für Inverse Kinematik berechnen (Roll, Pitch, Höhe)
                m_ikInput.pitch = -DegreeToRad(control_output_x_grad);
                m_ikInput.roll  = DegreeToRad(control_output_y_grad);
                m_ikInput.h     = 70.5f;


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
                    m_servo_commands[0] = DegreeToPWM(servo1_cmd_deg, BBOP_SERVO1_angle_range_grad);
                    m_servo_commands[1] = DegreeToPWM(servo2_cmd_deg, BBOP_SERVO2_angle_range_grad);
                    m_servo_commands[2] = DegreeToPWM(servo3_cmd_deg, BBOP_SERVO3_angle_range_grad);

                } else {
                    // Falls IK fehlschlägt: sicher auf Home-Lage zurück
                    m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad);
                    m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad);
                    m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad);
                }
            }  

        } else {

            // Ball weg, Servos in Mittelstellung halten
            m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad);
            m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad);
            m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad);
        }

        // Servo ansteuern
        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);
    }

    // Prepare next reply
    m_reply_data[0] = m_spiData.data[0]; // x ball camera [mm]
    m_reply_data[1] = m_spiData.data[1]; // y ball camera [mm]
    m_reply_data[2] = m_servo_commands[2]; // Echo servo D2 command
    m_reply_data[3] = m_ImuData.gyro.x();  // Gyro X in rad/sec
    m_reply_data[4] = m_ImuData.gyro.y();  // Gyro Y in rad/sec
    m_reply_data[5] = m_ImuData.gyro.z();  // Gyro Z in rad/sec
    m_reply_data[6] = m_ImuData.acc.x();   // Acc X in m/sec^2
    m_reply_data[7] = m_ImuData.acc.y();   // Acc Y in m/sec^2
    m_reply_data[8] = m_ImuData.acc.z();   // Acc Z in m/sec^2
    m_SpiSlaveDMA.setReplyData(m_reply_data, 9);

    // // Send data over serial stream (Kalman)
    // if (m_SerialStream.startByteReceived()) {
    //     m_SerialStream.write(dtime_us);            //  0 Delta time in us
    //     // m_SerialStream.write(m_servo_commands[0]); //  1 Echo servo D0 command
    //     // m_SerialStream.write(m_servo_commands[1]); //  2 Echo servo D1 command
    //     // m_SerialStream.write(m_servo_commands[2]); //  3 Echo servo D2 command
    //     // m_SerialStream.write(m_ImuData.gyro.x());  //  4 Gyro X in rad/sec
    //     // m_SerialStream.write(m_ImuData.gyro.y());  //  5 Gyro Y in rad/sec
    //     // m_SerialStream.write(m_ImuData.gyro.z());  //  6 Gyro Z in rad/sec
    //     m_SerialStream.write(m_ImuData.acc.x());   //  1 Acc X in m/sec^2
    //     m_SerialStream.write(m_ImuData.acc.y());   //  2 Acc Y in m/sec^2
    //     m_SerialStream.write(m_ImuData.acc.z());   //  3 Acc Z in m/sec^2
    //     m_SerialStream.write(m_ImuData.rpy.x());   // 4 Roll in rad
    //     m_SerialStream.write(m_ImuData.rpy.y());   // 5 Pitch in rad
    //     // m_SerialStream.write(m_ImuData.rpy.z());   // 6 Yaw in rad

    //     m_SerialStream.write(m_spiData.data[0]);              // 1 x_meas camera [mm]
    //     m_SerialStream.write(m_spiData.data[1]);              // 2 y_meas camera [mm]
    //     // m_SerialStream.write(m_spiData.data[2]);              // 3 z_meas camera [mm]

    //     m_SerialStream.write(m_kalmanX.getPositionMm());      // 4 x_hat [mm]
    //     m_SerialStream.write(m_kalmanX.getVelocityMmS());     // 5 vx_hat [mm/s]
    //     m_SerialStream.write(m_kalmanX.getDisturbanceRad());  // 6 dx_hat [rad]

    //     m_SerialStream.write(m_kalmanY.getPositionMm());      // 7 y_hat [mm]
    //     m_SerialStream.write(m_kalmanY.getVelocityMmS());     // 8 vy_hat [mm/s]
    //     m_SerialStream.write(m_kalmanY.getDisturbanceRad());  // 9 dy_hat [rad]

    //     m_SerialStream.write(newDataAvailable ? 1.0f : 0.0f); // 22 camera update flag
    //     m_SerialStream.write(m_kalmanHasFirstMeasurement ? 1.0f : 0.0f); // 23 kalman valid flag

    //     m_SerialStream.send();
        
    // }

    // Send data over serial stream (Trajectory)
    if (m_SerialStream.startByteReceived()) {
        m_SerialStream.write(dtime_us);            //  0 Delta time in us
        
        m_SerialStream.write(xd);                  //  1 x_des [mm]
        m_SerialStream.write(yd);                  //  2 y_des [mm]
        m_SerialStream.write(xd_dot);              //  3 vx_des [mm/s]
        m_SerialStream.write(yd_dot);              //  4 vy_des [mm/s]

        m_SerialStream.write(m_spiData.data[0]);   //  5 x_meas [mm]
        m_SerialStream.write(m_spiData.data[1]);   //  6 y_meas [mm]
        m_SerialStream.write(m_kalmanX.getPositionMm()); //  7 x_hat [mm]
        m_SerialStream.write(m_kalmanY.getPositionMm()); //  8 y_hat [mm]

        m_SerialStream.write(error_x);             //  9 error_x [mm]
        m_SerialStream.write(error_y);             // 10 error_y [mm]
        m_SerialStream.write(error_vx);            // 11 error_vx [mm/s]
        m_SerialStream.write(error_vy);            // 12 error_vy [mm/s]

        m_SerialStream.write(control_output_fb_x_grad); // 13 feedback x [deg]
        m_SerialStream.write(control_output_fb_y_grad); // 14 feedback y [deg]
        m_SerialStream.write(theta_ff_x_grad);          // 15 feedforward x [deg]
        m_SerialStream.write(theta_ff_y_grad);          // 16 feedforward y [deg]

        m_SerialStream.write(newDataAvailable ? 1.0f : 0.0f);             // 17 camera update
        m_SerialStream.write(m_kalmanHasFirstMeasurement ? 1.0f : 0.0f);  // 18 kalman valid

        m_SerialStream.send();
    }

    if (m_executeMain) {
        if (!m_servoD0.isEnabled()) {
            m_servoD0.enable(DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad));
        }
        if (!m_servoD1.isEnabled()) {
            m_servoD1.enable(DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad));
        }
        if (!m_servoD2.isEnabled()) {
            m_servoD2.enable(DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad));
        }
    } else {
        m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad);
        m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad);
        m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad);
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

float SPIComCntrl::DegreeToPWM(float degree, float range_degree)
{
    float pulse_width = (degree / range_degree);
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