#ifndef SPI_COM_CNTRL_H_
#define SPI_COM_CNTRL_H_

#include "IMU.h"
#include "PIDCntrl.h"
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
    //MKS
    static constexpr float SERVO_PULSE_MIN = 0.214f;
    static constexpr float SERVO_PULSE_MAX = 0.536f;
    // static constexpr float SERVO_PULSE_MIN = 0.214f;
    // static constexpr float SERVO_PULSE_MAX = 0.536f;
    static constexpr float BALL_POS_MIN_PX = -80.0f;
    static constexpr float BALL_POS_MAX_PX = 80.0f;
    static constexpr float BALL_POS_CENTER_PX = 0.0f;
    static constexpr float SERVO_CENTER = 0.5161f;
    static constexpr float ANGLE_DELTA_LIMIT_GRAD = 30.0f;
    static constexpr float BALL_CTRL_KP = 0.2f;
    static constexpr float BALL_CTRL_KI = 0.0f;
    static constexpr float BALL_CTRL_TAU_D_S = 0.0413f * 1.0f;
    static constexpr float BALL_CTRL_TAU_R_O = 0.000001f;
    static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (0.4157f - 0.0413f);
    static constexpr float VISION_TIMEOUT = 1.0F; // seconds

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
    PIDCntrl m_ballPosCntrl;

    float m_servo_commands[3]{};

    float m_reply_data[SPI_NUM_FLOATS];

    bool m_spi_ready{false};

    void executeTask() override;
    static float clamp(float val, float min, float max);
    static float clamp01(float val) { return clamp(val, 0.0f, 1.0f); }
    static float DegreeToPWM(float degree);
    static float PWMToDegree(float pulse_width);
    static float DegreeToRad(float degree);
};
#endif /* SPI_COM_CNTRL_H_ */
