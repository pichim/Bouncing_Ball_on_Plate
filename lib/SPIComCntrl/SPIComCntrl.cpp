#include "SPIComCntrl.h"

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

    // Read IMU data
    m_ImuData = m_Imu.getImuData();
    if (!m_Imu.isCalibrated())
        return;

    // Check for new SPI data from master
    if (m_SpiSlaveDMA.hasNewData()) {
        m_spiData = m_SpiSlaveDMA.getSPIData();

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

        // Update servo commands from SPI payload (first three floats expected in [0,1])
        m_servo_commands[0] = clamp01(m_spiData.data[0]);
        m_servo_commands[1] = clamp01(m_spiData.data[1]);
        m_servo_commands[2] = clamp01(m_spiData.data[2]);
        m_servoD0.setPulseWidth(m_servo_commands[0]);
        m_servoD1.setPulseWidth(m_servo_commands[1]);
        m_servoD2.setPulseWidth(m_servo_commands[2]);
    }

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
