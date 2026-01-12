#ifndef SPI_COM_CNTRL_H_
#define SPI_COM_CNTRL_H_

#include "IMU.h"
#include "RealTimeThread.h"
#include "SPISlaveDMA.h"
#include "SerialStream.h"
#include "Servo.h"
#include "config.h"
#include "mbed.h"

using namespace std::chrono;

class SPIComCntrl : public RealTimeThread
{
public:
    explicit SPIComCntrl();
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

    SerialStream m_SerialStream;
    Timer m_Timer;
    microseconds m_time_previous_us{0};

    float m_Ts;

    float m_servo_commands[3]{};

    float m_reply_data[SPI_NUM_FLOATS];

    bool m_spi_ready{false};

    void executeTask() override;
    static float clamp(float val, float min, float max);
    static float clamp01(float val) { return clamp(val, 0.0f, 1.0f); }
};
#endif /* SPI_COM_CNTRL_H_ */
