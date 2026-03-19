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
#include "Chirp.h"  // chirp signal generator


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
    // static constexpr float SERVO_PULSE_MIN = 0.0325f;
    // static constexpr float SERVO_PULSE_MAX = 0.1175f;
    //powerhd
    // static constexpr float SERVO_PULSE_MIN = 0.30175f;
    // static constexpr float SERVO_PULSE_MAX = 0.695f;   
    static constexpr float BALL_POS_MIN_PX = 0.0f;
    static constexpr float BALL_POS_MAX_PX = 1456.0f;
    static constexpr float BALL_POS_CENTER_PX = 728.0f;
    static constexpr float SERVO_CENTER = 0.5f;
    static constexpr float SERVO_DELTA_LIMIT = 0.1f;
    static constexpr float BALL_CTRL_KP = 0.5309f;
    static constexpr float BALL_CTRL_KI = 0.0f;
    static constexpr float BALL_CTRL_KD = 10.0f;
    static constexpr float BALL_CTRL_TAU_D_S = 0.1f;
    static constexpr float BALL_CTRL_TAU_R_O = 0.1f;

    // chirp parameters (start/end freq, duration) – sample period Ts set at runtime
    static constexpr float CHIRP_F0_HZ = 0.1f;
    static constexpr float CHIRP_F1_HZ = 245.0f;
    static constexpr float CHIRP_T1_S   = 100.0f;

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

    // chirp generator and control flag
    Chirp m_chirp;
    bool  m_start_chirp{false};
    float chirp_exc{0.0f}; // initial raw chirp excitation [-1,1]

    float m_servo_commands[3]{};

    float current_angle_deg{0.0f};

    float m_reply_data[SPI_NUM_FLOATS];

    bool m_spi_ready{false};

    void executeTask() override;
    static float clamp(float val, float min, float max);
    static float DegreeToPWM(float degree);
    static float PWMToDegree(float pulse_width);
    static float DegreeToRad(float degree);
    void toggle_servo();
    static float clamp01(float val) { return clamp(val, 0.0f, 1.0f); }
};
#endif /* SPI_COM_CNTRL_H_ */
