#include "SPIComCntrl.h"
#include <cmath>

// Servo & Inverse Kinematics mapping constants
namespace
{
    constexpr float IK_HOME_DEG = 90.0f;    // aus IK: roll=0, pitch=0, h=110.5
    constexpr float SERVO_MAX_DEG = 115.4f; // real nutzbarer Servobereich
    constexpr float SERVO_MIN_DEG = 0.0f;

    // Reale HOME-Winkel der 3 Servos bei waagerechter Platte
    constexpr float SERVO1_HOME_DEG = 55.0f + 0.5f;
    constexpr float SERVO2_HOME_DEG = 55.0f + 2.8f;
    constexpr float SERVO3_HOME_DEG = 55.0f - 2.1f;

    // gewünschte Begrenzung relativ zur Home-Lage (+/- 20°)
    constexpr float SERVO_CLAMP_DELTA_DEG = 20.0f;

    /*
     * executeTask läuft mit 1000 Hz.
     * Der Regler soll mit ca. 333 Hz laufen.
     * Deshalb wird der Regler nur jedes 3. Mal gerechnet.
     */
    constexpr int CONTROL_LOOP_DIVIDER = 3;

    constexpr float CONTROL_TS_S = CONTROL_LOOP_DIVIDER * BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US * 1.0e-6f;

    // Kamera läuft mit ca. 50 Hz
    constexpr float CAMERA_TS_S = 0.020f;

    // Kamera-Z-Grenze für gültige Ballmessung
    constexpr float CAMERA_Z_LIMIT_MM = 20.0f;

    float filtered_setpoint_x = 0.0f;
    float filtered_setpoint_y = 0.0f;

    // 333-Hz-Regler Counter
    int control_loop_counter = 0;

    // Kamera / Ball Status
    int missing_data_counter = 0;
    bool ballWasLost = true;

    // Letzter gültiger Kamerawert
    float last_x_meas_mm = 0.0f;
    float last_y_meas_mm = 0.0f;

    // Logging-Werte
    float log_error_x = 0.0f;
    float log_error_y = 0.0f;
    float log_control_output_x_grad = 0.0f;
    float log_control_output_y_grad = 0.0f;
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
    , m_observerX(static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f)
    , m_observerY(static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f)
    , m_Ts(static_cast<float>(BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US) * 1.0e-6f)
    , user_button(BBOP_USER_BUTTON, PullUp)
{
    // Start SPI communication
    if (!m_SpiSlaveDMA.start()) {
        printf("SPI start() failed — check wiring, pin mapping, or DMA state.\n");
        return;
    }

    m_spi_ready = true;
    printf("SPI Communication started. Waiting for master...\n");

    /*
     * Camera-only Positionsregler:
     * Der PID läuft mit 333 Hz, weil update() nur jedes 3. executeTask() aufgerufen wird.
     */
    m_ballPosCntrl_x.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, CONTROL_TS_S, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
    m_ballPosCntrl_y.setup(BALL_CTRL_KP, BALL_CTRL_KI, BALL_CTRL_KD, BALL_CTRL_TAU_f, BALL_CTRL_TAU_R_O, CONTROL_TS_S, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);

    m_ballPosCntrl_x.setIntegratorLimits(-ANGLE_DELTA_LIMIT_GRAD * 0.2f, ANGLE_DELTA_LIMIT_GRAD * 0.2f);
    m_ballPosCntrl_y.setIntegratorLimits(-ANGLE_DELTA_LIMIT_GRAD * 0.2f, ANGLE_DELTA_LIMIT_GRAD * 0.2f);

    // Trajectory setHold
    // m_trajectory.setHold(0.0f, 0.0f);

    // Trajectory setCircle
    // m_trajectory.setCircle(35.0f, 0.2f);

    // Trajectory setSequence
    static const SequencePoint seq[] = {
        {  0.0f,  0.0f, 6.0f },
        { 40.0f,  0.0f, 6.0f },
        {  0.0f,  0.0f, 6.0f },
        {-40.0f,  0.0f, 6.0f },
    };
    m_trajectory.setSequence(seq, 4);

    // Servo Kalibrierung
    m_servoD0.calibratePulseMinMax(SERVO1_PULSE_MIN, SERVO1_PULSE_MAX);
    m_servoD1.calibratePulseMinMax(SERVO2_PULSE_MIN, SERVO2_PULSE_MAX);
    m_servoD2.calibratePulseMinMax(SERVO3_PULSE_MIN, SERVO3_PULSE_MAX);

    // Initiale Servo-Kommandos auf reale Home-Lage setzen
    m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad);
    m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad);
    m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad);

    // Servos beim Start deaktiviert lassen
    m_servoD0.disable();
    m_servoD1.disable();
    m_servoD2.disable();

    m_Timer.start();

    // RealTimeThread::enable() muss nach Konstruktion extern aufgerufen werden
}

SPIComCntrl::~SPIComCntrl() = default;

void SPIComCntrl::executeTask()
{
    if (!m_spi_ready) {
        return;
    }
    
    // Button callback nur einmal registrieren
    if (!m_buttonCallbackAttached) {
        user_button.rise(callback(this, &SPIComCntrl::toggleExecuteMainFcn));
        m_buttonCallbackAttached = true;
    }
    
    // Zeit messen
    const microseconds time_us = m_Timer.elapsed_time();
    const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
    m_time_previous_us = time_us;

    // Trajektorie mit 1000 Hz aktualisieren
    TrajectoryRef traj = m_trajectory.update(m_Ts);

    // Sollwert weich filtern
    constexpr float SETPOINT_TAU_S = 0.7f * BALL_CTRL_TAU_V; // Tf aus MATLAB (Ca. 1 / w_d)
    const float alpha_sp = m_Ts / (SETPOINT_TAU_S + m_Ts);

    filtered_setpoint_x = filtered_setpoint_x + alpha_sp * (traj.x_mm - filtered_setpoint_x);
    filtered_setpoint_y = filtered_setpoint_y + alpha_sp * (traj.y_mm - filtered_setpoint_y);

    const float xd = filtered_setpoint_x;
    const float yd = filtered_setpoint_y;

    // IMU für Kalibrierstatus, Observer und Logging/Reply lesen
    m_ImuData = m_Imu.getImuData();

    bool newDataAvailable = false;
    bool validCameraUpdate = false;

    /*
     * Kamera / SPI lesen
     * Die Kamera liefert nur 50 Hz.
     * Wenn ein gültiger neuer Wert kommt, speichern wir ihn in last_x_meas_mm / last_y_meas_mm.
     * Zwischen den Kamera-Frames verwendet der 333-Hz-Regler diesen letzten gültigen Wert.
     */
    if (m_SpiSlaveDMA.hasNewData()) {

        newDataAvailable = true;
        m_spiData = m_SpiSlaveDMA.getSPIData();

        const float x_meas_mm = m_spiData.data[0];
        const float y_meas_mm = m_spiData.data[1];
        const float z_meas_mm = m_spiData.data[2];

        const bool cameraMeasurementValid = (z_meas_mm < CAMERA_Z_LIMIT_MM) && (z_meas_mm > -CAMERA_Z_LIMIT_MM);

        if (cameraMeasurementValid) {

            last_x_meas_mm = x_meas_mm;
            last_y_meas_mm = y_meas_mm;

            validCameraUpdate = true;
            missing_data_counter = 0;

            if (ballWasLost || !m_observerHasFirstMeasurement) {
                /*
                 * Ball wurde neu erkannt.
                 * Observer direkt auf aktuelle Kameraposition setzen.
                 */
                m_observerX.reset(last_x_meas_mm, 0.0f, 0.0f);
                m_observerY.reset(last_y_meas_mm, 0.0f, 0.0f);

                /*
                 * Regler zurücksetzen, damit kein alter Integratorwert übernommen wird.
                 */
                m_ballPosCntrl_x.reset(0.0f);
                m_ballPosCntrl_y.reset(0.0f);

                control_loop_counter = 0;

                log_error_x = 0.0f;
                log_error_y = 0.0f;
                log_control_output_x_grad = 0.0f;
                log_control_output_y_grad = 0.0f;

                m_observerHasFirstMeasurement = true;
            }

            ballWasLost = false;

        } else {

            /*
             * Neues SPI-Paket ist da, aber Kamera-Messung ist ungültig.
             * Dann gilt der Ball direkt als verloren.
             */
            missing_data_counter++;

            if (!ballWasLost) {
                m_ballPosCntrl_x.reset(0.0f);
                m_ballPosCntrl_y.reset(0.0f);
            }

            m_observerHasFirstMeasurement = false;
            ballWasLost = true;
            control_loop_counter = 0;
        }

    } else {

        /*
         * Kein neues Kamera-Paket.
         * Das ist normal, weil executeTask 1000 Hz läuft und Kamera nur 50 Hz.
         * Erst nach VISION_TIMEOUT gilt der Ball als verloren.
         */
        missing_data_counter++;

        if ((missing_data_counter * m_Ts) > VISION_TIMEOUT) {

            if (!ballWasLost) {
                m_ballPosCntrl_x.reset(0.0f);
                m_ballPosCntrl_y.reset(0.0f);
            }

            m_observerHasFirstMeasurement = false;
            ballWasLost = true;
            control_loop_counter = 0;
        }
    }

    /*
     * Observer läuft mit 1 kHz.
     * Die Kamera liefert nur ca. 50 Hz.
     * Zwischen zwei Kamera-Frames wird last_x_meas_mm / last_y_meas_mm gehalten.
     *
     * Wichtig:
     * Der Regler verwendet unten aktuell noch last_x_meas_mm / last_y_meas_mm.
     * Der Observer läuft hier nur im Hintergrund mit.
     */
    if (m_Imu.isCalibrated() && m_observerHasFirstMeasurement && !ballWasLost) {

        const float roll_rad  = m_ImuData.rpy.x();
        const float pitch_rad = m_ImuData.rpy.y();

        /*
         * x-Richtung:
         * u = pitch [rad]
         * y = letzte Kamera-x-Position [mm]
         */
        m_observerX.do_step(pitch_rad, last_x_meas_mm);

        /*
         * y-Richtung:
         * u = roll [rad]
         * y = letzte Kamera-y-Position [mm]
         */
        m_observerY.do_step(roll_rad, last_y_meas_mm);
    }

    /*
     * Standard Home-Kommandos, falls Regler nicht aktiv ist oder IK fehlschlägt.
     */
    const float servo1_home_pwm = DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad);
    const float servo2_home_pwm = DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad);
    const float servo3_home_pwm = DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad);

    /*
     * Hauptlogik
     */
    if (!m_Imu.isCalibrated()) {

        // Solange IMU nicht kalibriert ist: Home
        m_servo_commands[0] = servo1_home_pwm;
        m_servo_commands[1] = servo2_home_pwm;
        m_servo_commands[2] = servo3_home_pwm;

    } else if (!m_executeMain) {

        // Regler ausgeschaltet: Home halten
        m_servo_commands[0] = servo1_home_pwm;
        m_servo_commands[1] = servo2_home_pwm;
        m_servo_commands[2] = servo3_home_pwm;

        control_loop_counter = 0;

        log_error_x = 0.0f;
        log_error_y = 0.0f;
        log_control_output_x_grad = 0.0f;
        log_control_output_y_grad = 0.0f;

    } else if (ballWasLost || !m_observerHasFirstMeasurement) {

        // Ball weg: Servos deaktivieren
        m_servo_commands[0] = servo1_home_pwm;
        m_servo_commands[1] = servo2_home_pwm;
        m_servo_commands[2] = servo3_home_pwm;

        // Servos aktiv lassen / wieder aktivieren, damit die Platte wirklich auf Home fährt
        if (!m_servoD0.isEnabled()) {
            m_servoD0.enable(servo1_home_pwm);
        }
        if (!m_servoD1.isEnabled()) {
            m_servoD1.enable(servo2_home_pwm);
        }
        if (!m_servoD2.isEnabled()) {
            m_servoD2.enable(servo3_home_pwm);
        }

        // Regler zurücksetzen
        m_ballPosCntrl_x.reset(0.0f);
        m_ballPosCntrl_y.reset(0.0f);

        control_loop_counter = 0;

        log_error_x = 0.0f;
        log_error_y = 0.0f;
        log_control_output_x_grad = 0.0f;
        log_control_output_y_grad = 0.0f;

    } else {

        /*
         * Ball ist vorhanden und Regler ist aktiv.
         * Servos wieder aktivieren, falls sie vorher deaktiviert waren.
         */
        if (!m_servoD0.isEnabled()) {
            m_servoD0.enable(m_servo_commands[0]);
        }
        if (!m_servoD1.isEnabled()) {
            m_servoD1.enable(m_servo_commands[1]);
        }
        if (!m_servoD2.isEnabled()) {
            m_servoD2.enable(m_servo_commands[2]);
        }

        /*
         * Regler nur jedes 3. executeTask() rechnen.
         * executeTask: 1000 Hz
         * Regler: ca. 333 Hz
         */
        control_loop_counter++;

        if (control_loop_counter >= CONTROL_LOOP_DIVIDER) {

            control_loop_counter = 0;

            // Camera-only Positionsfehler
            const float error_x = xd - last_x_meas_mm;
            const float error_y = yd - last_y_meas_mm;
            
            // // Statischer Kalman Positionsfehler
            // const float error_x = xd - m_observerX.getPositionMm();
            // const float error_y = yd - m_observerY.getPositionMm();

            // PID-T1 Positionsregler
            float control_output_x_grad = m_ballPosCntrl_x.update(error_x);
            float control_output_y_grad = m_ballPosCntrl_y.update(error_y);

            // Sicherheitsbegrenzung
            control_output_x_grad = clamp(control_output_x_grad, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);
            control_output_y_grad = clamp(control_output_y_grad, -ANGLE_DELTA_LIMIT_GRAD, ANGLE_DELTA_LIMIT_GRAD);

            // Für Logging speichern
            log_error_x = error_x;
            log_error_y = error_y;
            log_control_output_x_grad = control_output_x_grad;
            log_control_output_y_grad = control_output_y_grad;

            /*
             * Inputs für inverse Kinematik
             * x-Regler -> Pitch
             * y-Regler -> Roll mit negativem Vorzeichen
             */
            m_ikInput.pitch = DegreeToRad(control_output_x_grad);
            m_ikInput.roll  = -DegreeToRad(control_output_y_grad);
            m_ikInput.h     = 110.5f;

            InverseKinematics3Leg::Result ikResult = m_ik.compute(m_ikInput);

            if (ikResult.success) {

                /*
                 * IK-Winkel relativ zur IK-Home-Lage auf reale Servo-Home-Lage addieren.
                 */
                float servo1_cmd_deg = SERVO1_HOME_DEG + (ikResult.alphaDeg[0] - IK_HOME_DEG);
                float servo2_cmd_deg = SERVO2_HOME_DEG + (ikResult.alphaDeg[1] - IK_HOME_DEG);
                float servo3_cmd_deg = SERVO3_HOME_DEG + (ikResult.alphaDeg[2] - IK_HOME_DEG);

                // Clamp auf +/-20° um die jeweilige reale Home-Lage
                servo1_cmd_deg = clamp(servo1_cmd_deg, SERVO1_HOME_DEG - SERVO_CLAMP_DELTA_DEG, SERVO1_HOME_DEG + SERVO_CLAMP_DELTA_DEG);
                servo2_cmd_deg = clamp(servo2_cmd_deg, SERVO2_HOME_DEG - SERVO_CLAMP_DELTA_DEG, SERVO2_HOME_DEG + SERVO_CLAMP_DELTA_DEG);
                servo3_cmd_deg = clamp(servo3_cmd_deg, SERVO3_HOME_DEG - SERVO_CLAMP_DELTA_DEG, SERVO3_HOME_DEG + SERVO_CLAMP_DELTA_DEG);

                // Harter Sicherheitsclamp auf realen Servo-Bereich
                servo1_cmd_deg = clamp(servo1_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
                servo2_cmd_deg = clamp(servo2_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
                servo3_cmd_deg = clamp(servo3_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);

                // In PWM umrechnen
                m_servo_commands[0] = DegreeToPWM(servo1_cmd_deg, BBOP_SERVO1_angle_range_grad);
                m_servo_commands[1] = DegreeToPWM(servo2_cmd_deg, BBOP_SERVO2_angle_range_grad);
                m_servo_commands[2] = DegreeToPWM(servo3_cmd_deg, BBOP_SERVO3_angle_range_grad);

            } else {

                // Falls IK fehlschlägt: Home
                m_servo_commands[0] = servo1_home_pwm;
                m_servo_commands[1] = servo2_home_pwm;
                m_servo_commands[2] = servo3_home_pwm;
            }
        }
    }

    /*
     * Servo-Pulse setzen.
     * Wenn Servo disabled ist, sollte setPulseWidth je nach Servo-Klasse keinen Effekt haben.
     * Wenn er enabled ist, wird das letzte gültige Kommando gehalten.
     */
    m_servoD0.setPulseWidth(m_servo_commands[0]);
    m_servoD1.setPulseWidth(m_servo_commands[1]);
    m_servoD2.setPulseWidth(m_servo_commands[2]);

    /*
     * SPI Reply
     */
    m_reply_data[0] = last_x_meas_mm;        // x ball camera [mm]
    m_reply_data[1] = last_y_meas_mm;        // y ball camera [mm]
    m_reply_data[2] = m_servo_commands[2];   // Echo servo D2 command
    m_reply_data[3] = m_ImuData.gyro.x();    // Gyro X [rad/s]
    m_reply_data[4] = m_ImuData.gyro.y();    // Gyro Y [rad/s]
    m_reply_data[5] = m_ImuData.gyro.z();    // Gyro Z [rad/s]
    m_reply_data[6] = m_ImuData.acc.x();     // Acc X [m/s^2]
    m_reply_data[7] = m_ImuData.acc.y();     // Acc Y [m/s^2]
    m_reply_data[8] = m_ImuData.acc.z();     // Acc Z [m/s^2]

    m_SpiSlaveDMA.setReplyData(m_reply_data, 9);

    /*
     * Camera + Observer Logging
     */
    if (m_SerialStream.startByteReceived()) {

        m_SerialStream.write(dtime_us);                         //  0 Delta time [us] -> data.time

        m_SerialStream.write(xd);                               //  1 x_des [mm]
        m_SerialStream.write(yd);                               //  2 y_des [mm]

        m_SerialStream.write(last_x_meas_mm);                   //  3 x_meas camera [mm]
        m_SerialStream.write(last_y_meas_mm);                   //  4 y_meas camera [mm]

        m_SerialStream.write(m_observerX.getPositionMm());      //  5 x_hat observer [mm]
        m_SerialStream.write(m_observerY.getPositionMm());      //  6 y_hat observer [mm]

        m_SerialStream.write(m_observerX.getVelocityMmS());     //  7 vx_hat observer [mm/s]
        m_SerialStream.write(m_observerY.getVelocityMmS());     //  8 vy_hat observer [mm/s]

        m_SerialStream.write(m_observerX.getDisturbanceRad());  //  9 disturbance_x [rad]
        m_SerialStream.write(m_observerY.getDisturbanceRad());  // 10 disturbance_y [rad]

        m_SerialStream.write(log_error_x);                      // 11 error_x [mm]
        m_SerialStream.write(log_error_y);                      // 12 error_y [mm]

        m_SerialStream.write(log_control_output_x_grad);        // 13 controller output x [deg]
        m_SerialStream.write(log_control_output_y_grad);        // 14 controller output y [deg]

        m_SerialStream.write(newDataAvailable ? 1.0f : 0.0f);   // 15 new SPI data flag
        m_SerialStream.write(validCameraUpdate ? 1.0f : 0.0f);  // 16 valid camera update flag
        m_SerialStream.write(ballWasLost ? 1.0f : 0.0f);        // 17 ball lost flag
        m_SerialStream.write(m_executeMain ? 1.0f : 0.0f);      // 18 execute main flag
        m_SerialStream.write(m_observerHasFirstMeasurement ? 1.0f : 0.0f); // 19 observer valid flag

        m_SerialStream.send();
    }

}

void SPIComCntrl::toggleExecuteMainFcn()
{
    m_executeMain = !m_executeMain;
}

float SPIComCntrl::clamp(float val, float min, float max)
{
    if (val < min) {
        return min;
    }

    if (val > max) {
        return max;
    }

    return val;
}

float SPIComCntrl::DegreeToPWM(float degree, float range_degree)
{
    return degree / range_degree;
}

float SPIComCntrl::PWMToDegree(float pulse_width)
{
    return pulse_width * SERVO_MAX_DEG;
}

float SPIComCntrl::DegreeToRad(float degree)
{
    return degree * (PI / 180.0f);
}