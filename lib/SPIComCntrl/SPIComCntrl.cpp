#include "SPIComCntrl.h"
#include "InverseKinematics3Leg.h"

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
        printf("SPI start() failed — check wiring, pin mapping, or DMA state.\n");
        // Für reinen Servo-Test könnte man hier auch trotzdem weitermachen
    } else {
        m_spi_ready = true;
        printf("SPI Communication started. Waiting for master...\n");
    }

    // Servo calibration
    m_servoD0.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD1.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD2.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);

    // Servos auf Mittelstellung aktivieren
    if (!m_servoD0.isEnabled()) {
        m_servoD0.enable(0.387f);
    }
    if (!m_servoD1.isEnabled()) {
        m_servoD1.enable(0.370f);
    }
    if (!m_servoD2.isEnabled()) {
        m_servoD2.enable(0.382f);
    }

    m_servo_commands[0] = 0.387f;
    m_servo_commands[1] = 0.370f;
    m_servo_commands[2] = 0.382f;

    m_Timer.start();

    printf("SPIComCntrl test mode initialized.\n");
}

SPIComCntrl::~SPIComCntrl() = default;


// Global / file-local IK object like in your style
InverseKinematics3Leg ik;
InverseKinematics3Leg::Input ikInput;

void SPIComCntrl::executeTask()
{
    // ============================================================
    // TEST MODE:
    // Fixed platform pose -> IK -> Servo commands
    // No ball controller, no trajectory, no vision needed
    // ============================================================

    // Example test pose
    ikInput.roll  = DegreeToRad(0.0f);      // [rad]
    ikInput.pitch = DegreeToRad(0.0f);    // [rad]
    ikInput.h     = 70.0f;                  // [mm]

    // Example alternative test cases:
    // ikInput.roll  = DegreeToRad(5.0f);
    // ikInput.pitch = DegreeToRad(0.0f);
    // ikInput.h     = 98.1f;

    // ikInput.roll  = DegreeToRad(0.0f);
    // ikInput.pitch = DegreeToRad(5.0f);
    // ikInput.h     = 98.1f;

    InverseKinematics3Leg::Result ikResult = ik.compute(ikInput);

    if (!ikResult.success) {
        printf("IK computation failed: %s\n", ikResult.errorMessage.c_str());

        // Safe fallback: hold center position
        m_servo_commands[0] = 0.387f;
        m_servo_commands[1] = 0.370f;
        m_servo_commands[2] = 0.382f;

        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);
        return;
    }

    printf("IK successful | alphaDeg: [%.2f, %.2f, %.2f] | rodError: [%.2f, %.2f, %.2f]\n",
           ikResult.alphaDeg[0], ikResult.alphaDeg[1], ikResult.alphaDeg[2],
           ikResult.rodError[0], ikResult.rodError[1], ikResult.rodError[2]);

    // ------------------------------------------------------------
    // IMPORTANT:
    // Here we assume that alpha = 90 deg corresponds roughly to the
    // mechanical neutral pose (servo horn horizontal outward).
    // That is why we subtract 90 deg.
    //
    // If your real neutral pose is different, change this offset.
    // ------------------------------------------------------------
    float control_output_1 = DegreeToPWM(ikResult.alphaDeg[0] - 90.0f);
    float control_output_2 = DegreeToPWM(ikResult.alphaDeg[1] - 90.0f);
    float control_output_3 = DegreeToPWM(ikResult.alphaDeg[2] - 90.0f);

    float delta20 = DegreeToPWM(20.0f);

    m_servo_commands[0] = clamp(0.387f + control_output_1, 0.387f - delta20, 0.387f + delta20);
    m_servo_commands[1] = clamp(0.370f + control_output_2, 0.370f - delta20, 0.370f + delta20);
    m_servo_commands[2] = clamp(0.382f + control_output_3, 0.382f - delta20, 0.382f + delta20);

    printf("Servo Commands (PWM): D0: %.3f | D1: %.3f | D2: %.3f\n",
           m_servo_commands[0],
           m_servo_commands[1],
           m_servo_commands[2]);

    // Send commands to servos
    m_servoD0.setPulseWidth(m_servo_commands[0]);
    m_servoD1.setPulseWidth(m_servo_commands[1]);
    m_servoD2.setPulseWidth(m_servo_commands[2]);
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
    float pulse_width = (degree / BBOP_SERVO_angle_range_grad);
    return pulse_width;
}

float SPIComCntrl::PWMToDegree(float pulse_width)
{
    float degree = pulse_width * BBOP_SERVO_angle_range_grad;
    return degree;
}

float SPIComCntrl::DegreeToRad(float degree)
{
    float rad = degree * (M_PIf / 180.0f);
    return rad;
}