#include "SPIComCntrl.h"
#include "InverseKinematics3Leg.h"
#include "KalmanBall1D.h"
#include <chrono>

// Servo & Inverse Kinematics mapping constants
namespace
{
    constexpr float IK_HOME_DEG = 90.0f;    // aus IK: roll=0, pitch=0, h=110.5
    constexpr float SERVO_MAX_DEG = 122.7f; // real nutzbarer Servobereich
    constexpr float SERVO_MIN_DEG = 0.0f;

    // Reale HOME-Winkel der 3 Servos bei waagerechter Platte
    constexpr float SERVO1_HOME_DEG = 90.0f + 6.5f; // 96.5°
    constexpr float SERVO2_HOME_DEG = 90.0f + 9.0f; // 99.0°
    constexpr float SERVO3_HOME_DEG = 90.0f - 0.5f; // 89.5°

    // gewünschte Begrenzung relativ zur Home-Lage (+/- 20°)
    constexpr float SERVO_CLAMP_DELTA_DEG = 20.0f;
}

// Global / file-local IK object like in your style
InverseKinematics3Leg ik;
InverseKinematics3Leg::Input ikInput;

KalmanBall1D m_kalmanX;
KalmanBall1D m_kalmanY;

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
    // Optional: SPI starten, falls du es im restlichen Projekt brauchst
    if (!m_SpiSlaveDMA.start()) {
        // printf("SPI start() failed — check wiring, pin mapping, or DMA state.\n");
    } else {
        m_spi_ready = true;
        // printf("SPI Communication started. Waiting for master...\n");
    }

    // Servo calibration
    m_servoD0.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD1.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD2.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);

    // Servos auf Mittelstellung aktivieren
    m_servoD0.enable(DegreeToPWM(SERVO1_HOME_DEG));
    m_servoD1.enable(DegreeToPWM(SERVO2_HOME_DEG));
    m_servoD2.enable(DegreeToPWM(SERVO3_HOME_DEG));

    // Initiale Befehle auf Home-Position setzen
    m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG);
    m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG);
    m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG);

    m_Timer.start();

    // printf("SPIComCntrl test mode initialized.\n");
}

SPIComCntrl::~SPIComCntrl() = default;

void SPIComCntrl::executeTask()
{
    // ============================================================
    // Zeitmessung für MATLAB-Logging
    // ============================================================
    static auto time_previous_us = m_Timer.elapsed_time();
    const auto time_us = m_Timer.elapsed_time();

    const float dtime_us = static_cast<float>(
        std::chrono::duration_cast<std::chrono::microseconds>(time_us - time_previous_us).count());

    time_previous_us = time_us;

    // ============================================================
    // IMU lesen
    // ============================================================
    ImuData imuData = m_Imu.getImuData();
    if (!m_Imu.isCalibrated()) {
        // printf("IMU calibrating... %u/%u\n", m_avg_cntr, BBOP_IMU_NUM_RUNS_FOR_AVERAGE);
    }
    
    // ============================================================
    // TEST MODE:
    // Fixed platform pose -> IK -> Servo commands
    // ============================================================
    ikInput.roll  = DegreeToRad(0.0f);
    ikInput.pitch = DegreeToRad(0.0f);
    ikInput.h     = 110.5f;

    InverseKinematics3Leg::Result ikResult = ik.compute(ikInput);

    if (!ikResult.success) {
        m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG);
        m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG);
        m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG);

        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);
    } else {
        float servo1_cmd_deg = SERVO1_HOME_DEG - (ikResult.alphaDeg[0] - IK_HOME_DEG);
        float servo2_cmd_deg = SERVO2_HOME_DEG - (ikResult.alphaDeg[1] - IK_HOME_DEG);
        float servo3_cmd_deg = SERVO3_HOME_DEG - (ikResult.alphaDeg[2] - IK_HOME_DEG);

        servo1_cmd_deg = clamp(servo1_cmd_deg,
                               SERVO1_HOME_DEG - SERVO_CLAMP_DELTA_DEG,
                               SERVO1_HOME_DEG + SERVO_CLAMP_DELTA_DEG);

        servo2_cmd_deg = clamp(servo2_cmd_deg,
                               SERVO2_HOME_DEG - SERVO_CLAMP_DELTA_DEG,
                               SERVO2_HOME_DEG + SERVO_CLAMP_DELTA_DEG);

        servo3_cmd_deg = clamp(servo3_cmd_deg,
                               SERVO3_HOME_DEG - SERVO_CLAMP_DELTA_DEG,
                               SERVO3_HOME_DEG + SERVO_CLAMP_DELTA_DEG);

        servo1_cmd_deg = clamp(servo1_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
        servo2_cmd_deg = clamp(servo2_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
        servo3_cmd_deg = clamp(servo3_cmd_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);

        m_servo_commands[0] = DegreeToPWM(servo1_cmd_deg);
        m_servo_commands[1] = DegreeToPWM(servo2_cmd_deg);
        m_servo_commands[2] = DegreeToPWM(servo3_cmd_deg);

        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);
    }

    // ============================================================
    // UART / SerialStream Export für MATLAB
    // ============================================================
    if (m_SerialStream.startByteReceived()) {
        m_SerialStream.write(dtime_us);          //  0 Delta time in us
        m_SerialStream.write(m_servo_commands[0]); //  1 Servo 1 PWM normiert
        m_SerialStream.write(m_servo_commands[1]); //  2 Servo 2 PWM normiert
        m_SerialStream.write(m_servo_commands[2]); //  3 Servo 3 PWM normiert

        m_SerialStream.write(imuData.gyro.x());  //  4 Gyro X [rad/s]
        m_SerialStream.write(imuData.gyro.y());  //  5 Gyro Y [rad/s]
        m_SerialStream.write(imuData.gyro.z());  //  6 Gyro Z [rad/s]

        m_SerialStream.write(imuData.acc.x());   //  7 Acc X [m/s^2]
        m_SerialStream.write(imuData.acc.y());   //  8 Acc Y [m/s^2]
        m_SerialStream.write(imuData.acc.z());   //  9 Acc Z [m/s^2]

        m_SerialStream.write(imuData.rpy.x());   // 10 Roll  [rad]
        m_SerialStream.write(imuData.rpy.y());   // 11 Pitch [rad]
        m_SerialStream.write(imuData.rpy.z());   // 12 Yaw   [rad]

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