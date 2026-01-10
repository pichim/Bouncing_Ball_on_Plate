#include "SPIComCntrl.h"

SPIComCntrl::SPIComCntrl(PinName spi_mosi_pin,
                         PinName spi_miso_pin,
                         PinName spi_sck_pin,
                         PinName spi_nss_pin,
                         osPriority spi_priority,
                         uint32_t spi_stack_size,
                         PinName imu_sda_pin,
                         PinName imu_scl_pin,
                         PinName servo_d0_pin,
                         PinName servo_d1_pin,
                         PinName servo_d2_pin,
                         uint32_t period_us,
                         osPriority priority,
                         uint32_t stack_size)
    : RealTimeThread(period_us, priority, stack_size)
    , m_SpiSlaveDMA(spi_mosi_pin, spi_miso_pin, spi_sck_pin, spi_nss_pin, spi_priority, spi_stack_size)
    , m_Imu(imu_sda_pin, imu_scl_pin)
    , m_servoD0(servo_d0_pin)
    , m_servoD1(servo_d1_pin)
    , m_servoD2(servo_d2_pin)
    , m_Ts(static_cast<float>(period_us) * 1.0e-6f)
{
    // Start SPI communication; guard failure
    if (!m_SpiSlaveDMA.start()) {
        printf("SPI start() failed — check wiring, pin mapping, or DMA state.\n");
        return;
    }

    m_spi_ready = true;
    printf("SPI Communication started. Waiting for master...\n");

    // Calibrate and enable servos (normalised pulse widths)
    m_servoD0.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD1.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    m_servoD2.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);

    if (!m_servoD0.isEnabled()) {
        m_servoD0.enable();
    }
    if (!m_servoD1.isEnabled()) {
        m_servoD1.enable();
    }
    if (!m_servoD2.isEnabled()) {
        m_servoD2.enable();
    }

    // NOTE: RealTimeThread::enable() must be called by the user after construction is complete
}

SPIComCntrl::~SPIComCntrl() = default;

void SPIComCntrl::executeTask()
{
    if (!m_spi_ready) {
        return;
    }

    // Check for new SPI data from master
    if (m_SpiSlaveDMA.hasNewData()) {
        m_spiData = m_SpiSlaveDMA.getSPIData();

        printf("Message: %lu | Delta Time: %lu us | "
               "Received: [%.2f, %.2f, %.2f] | "
               "Header: 0x%02X | Failed: %lu | "
               "Readout Time: %lu us\n",
               m_spiData.message_count,
               m_spiData.last_delta_time_us,
               m_spiData.data[0],
               m_spiData.data[1],
               m_spiData.data[2],
               SPI_HEADER_SLAVE,
               m_spiData.failed_count,
               m_spiData.readout_time_us);

        // Update servo commands from SPI payload (first three floats expected in [0,1])
        m_servo_commands[0] = clamp01(m_spiData.data[0]);
        m_servo_commands[1] = clamp01(m_spiData.data[1]);
        m_servo_commands[2] = clamp01(m_spiData.data[2]);
        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);
    }

    // Read IMU data
    m_ImuData = m_Imu.getImuData();

    // Prepare next reply
    m_reply_data[0] = m_servo_commands[0]; // Echo servo D0 command
    m_reply_data[1] = m_servo_commands[1]; // Echo servo D1 command
    m_reply_data[2] = m_servo_commands[2]; // Echo servo D2 command
    m_reply_data[3] = m_ImuData.gyro.x();  // Gyro X in rad/sec
    m_reply_data[4] = m_ImuData.gyro.y();  // Gyro Y in rad/sec
    m_reply_data[5] = m_ImuData.gyro.z();  // Gyro Z in rad/sec
    m_reply_data[6] = m_ImuData.acc.x();   // Acc X in m/sec^2
    m_reply_data[7] = m_ImuData.acc.y();   // Acc Y in m/sec^2
    m_reply_data[8] = m_ImuData.acc.z();   // Acc Z in m/sec^2
    m_reply_data[9] = m_ImuData.mag.x();   // Mag X
    m_reply_data[10] = m_ImuData.mag.y();  // Mag Y
    m_reply_data[11] = m_ImuData.mag.z();  // Mag Z
    m_SpiSlaveDMA.setReplyData(m_reply_data, 12);
}

float SPIComCntrl::clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}
