#include "SPIComCntrl.h"
#include "InverseKinematics3Leg.h"
#include "KalmanBall1D.h"
#include <chrono>

// Servo & Inverse Kinematics mapping constants
namespace
{
    constexpr float IK_HOME_DEG = 90.0f;    // aus IK: roll=0, pitch=0, h=110.5
    constexpr float SERVO_MAX_DEG = 115.4f; // real nutzbarer Servobereich
    constexpr float SERVO_MIN_DEG = 0.0f;

    // Reale HOME-Winkel der 3 Servos bei waagerechter Platte
    constexpr float SERVO1_HOME_DEG = 55.0f + 1.9f;
    constexpr float SERVO2_HOME_DEG = 55.0f + 2.8f;
    constexpr float SERVO3_HOME_DEG = 55.0f - 1.6f;

    // gewünschte Begrenzung relativ zur Home-Lage (+/- 25°)
    constexpr float SERVO_CLAMP_DELTA_DEG = 25.0f;
}

// Global / file-local IK object like in your style
InverseKinematics3Leg ik;
InverseKinematics3Leg::Input ikInput;

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
    m_servoD0.calibratePulseMinMax(SERVO1_PULSE_MIN, SERVO1_PULSE_MAX);
    m_servoD1.calibratePulseMinMax(SERVO2_PULSE_MIN, SERVO2_PULSE_MAX);
    m_servoD2.calibratePulseMinMax(SERVO3_PULSE_MIN, SERVO3_PULSE_MAX);

    // Servos auf Mittelstellung aktivieren
    m_servoD0.enable(DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad));
    m_servoD1.enable(DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad));
    m_servoD2.enable(DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad));

    // Initiale Befehle auf Home-Position setzen
    m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad);
    m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad);
    m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad);

    m_Timer.start();
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

    // ============================================================
    // TEST MODE:
    // Roll step sequence starts only after IMU calibration
    // ============================================================

    constexpr int STEP_HOLD_CYCLES = 4000; // 5000 * 1 ms = 5 s

    static const float roll_steps_deg[] = {
        0.0f,
        -5.0f,
        -10.0f,
        -15.0f,
        -18.0f,
        0.0f,
        5.0f,
        10.0f,
        15.0f,
        18.0f
    };

    static constexpr int NUM_ROLL_STEPS =
        sizeof(roll_steps_deg) / sizeof(roll_steps_deg[0]);

    static bool sequence_started = false;
    static int step_index = 0;
    static int step_counter = 0;

    float roll_cmd_deg = 0.0f;

    // While IMU is not calibrated, stay at 0°
    if (!m_Imu.isCalibrated()) {

        sequence_started = false;
        step_index = 0;
        step_counter = 0;

        roll_cmd_deg = 0.0f;
    }
    else {

        // Start sequence after IMU calibration
        if (!sequence_started) {
            sequence_started = true;
            step_index = 0;
            step_counter = 0;
        }

        roll_cmd_deg = roll_steps_deg[step_index];

        step_counter++;

        if (step_counter >= STEP_HOLD_CYCLES) {
            step_counter = 0;

            if (step_index < NUM_ROLL_STEPS - 1) {
                step_index++;
            }
        }
    }

    // Set IK input a
    ikInput.roll  = DegreeToRad(10.0f); // x-achse
    ikInput.pitch = DegreeToRad(0.0f);  // y-achse
    ikInput.h     = 110.5f;

    InverseKinematics3Leg::Result ikResult = ik.compute(ikInput);

    if (!ikResult.success) {
        m_servo_commands[0] = DegreeToPWM(SERVO1_HOME_DEG, BBOP_SERVO1_angle_range_grad);
        m_servo_commands[1] = DegreeToPWM(SERVO2_HOME_DEG, BBOP_SERVO2_angle_range_grad);
        m_servo_commands[2] = DegreeToPWM(SERVO3_HOME_DEG, BBOP_SERVO3_angle_range_grad);

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

        m_servo_commands[0] = DegreeToPWM(servo1_cmd_deg, BBOP_SERVO1_angle_range_grad);
        m_servo_commands[1] = DegreeToPWM(servo2_cmd_deg, BBOP_SERVO2_angle_range_grad);
        m_servo_commands[2] = DegreeToPWM(servo3_cmd_deg, BBOP_SERVO3_angle_range_grad);

        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);
    }

    // ============================================================
    // UART / SerialStream Export für MATLAB
    // ============================================================
    if (m_SerialStream.startByteReceived()) {
        m_SerialStream.write(dtime_us);          //  0 Delta time in us
        
        m_SerialStream.write(ikInput.roll);      // 1 Roll soll [rad]
        m_SerialStream.write(ikInput.pitch);     // 1 Pitch ist [rad]
        m_SerialStream.write(ikInput.h);         // 2 Höhe      [mm]
        m_SerialStream.write(imuData.rpy.x());   // 3 Roll IMU  [rad]
        m_SerialStream.write(imuData.rpy.y());   // 4 Pitch IMU [rad]

        m_SerialStream.send();
    }
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
    return degree * BBOP_DEG_TO_RAD;
}