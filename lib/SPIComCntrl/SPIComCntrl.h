#ifndef SPI_COM_CNTRL_H_
#define SPI_COM_CNTRL_H_

#include "mbed.h"

// #include "PESBoardPinMap.h"

#include "IMU.h"
#include "RealTimeThread.h"
#include "SPISlaveDMA.h"
#include "Servo.h"

class SPIComCntrl : public RealTimeThread
{
public:
    explicit SPIComCntrl(PinName spi_mosi_pin,
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
                         uint32_t stack_size);
    virtual ~SPIComCntrl();

private:
    static constexpr float SERVO_PULSE_MIN = 0.0325f;
    static constexpr float SERVO_PULSE_MAX = 0.1175f;

    SpiData m_spiData;
    SpiSlaveDMA m_SpiSlaveDMA;

    ImuData m_ImuData;
    IMU m_Imu;

    Servo m_servoD0;
    Servo m_servoD1;
    Servo m_servoD2;

    float m_Ts;

    float m_servo_commands[3]{};

    float m_reply_data[SPI_NUM_FLOATS];

    bool m_spi_ready{false};

    void executeTask() override;
    static float clamp(float val, float min, float max);
    static float clamp01(float val) { return clamp(val, 0.0f, 1.0f); }
};
#endif /* SPI_COM_CNTRL_H_ */
