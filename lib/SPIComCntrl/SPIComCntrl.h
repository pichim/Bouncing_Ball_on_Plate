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
#include "InverseKinematics3Leg.h"
#include "DebounceIn.h"
#include "TrajectoryGenerator.h"
#include "observer.h"

using namespace std::chrono;

class SPIComCntrl : public RealTimeThread
{
public:
    explicit SPIComCntrl();
    virtual ~SPIComCntrl();

private:
    // PowerHD 1
    static constexpr float SERVO1_PULSE_MIN = 0.3f;
    static constexpr float SERVO1_PULSE_MAX = 0.695f;  

    // PowerHD 2
    static constexpr float SERVO2_PULSE_MIN = 0.305f;
    static constexpr float SERVO2_PULSE_MAX = 0.7025f;  

    // PowerHD 3
    static constexpr float SERVO3_PULSE_MIN = 0.3025f;
    static constexpr float SERVO3_PULSE_MAX = 0.6975f; 

    static constexpr float ANGLE_DELTA_LIMIT_GRAD = 20.0f;

    // 333 hz ohne Kalman
    static constexpr float BALL_CTRL_KP = 0.05f;
    static constexpr float BALL_CTRL_KI = 0.004f;
    static constexpr float BALL_CTRL_TAU_V = 0.9f;
    static constexpr float BALL_CTRL_TAU_f = 0.1f;
    static constexpr float BALL_CTRL_TAU_R_O = 0.01f;
    static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    // // 333 hz mit Kalman
    // static constexpr float BALL_CTRL_KP = 0.05f;
    // static constexpr float BALL_CTRL_KI = 0.004f;
    // static constexpr float BALL_CTRL_TAU_V = 0.9f;
    // static constexpr float BALL_CTRL_TAU_f = 0.1f;
    // static constexpr float BALL_CTRL_TAU_R_O = 0.01f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    static constexpr float PI = 3.14159265358979323846f;

    SpiData m_spiData;
    SpiSlaveDMA m_SpiSlaveDMA;

    ImuData m_ImuData;
    IMU m_Imu;

    Servo m_servoD0;
    Servo m_servoD1;
    Servo m_servoD2;

    SerialStream m_SerialStream;

    observer m_observerX;
    observer m_observerY;

    Timer m_Timer;
    microseconds m_time_previous_us{0};

    InverseKinematics3Leg m_ik;
    InverseKinematics3Leg::Input m_ikInput;

    TrajectoryGenerator m_trajectory;

    float m_Ts;

    PIDCntrl m_ballPosCntrl_x;
    PIDCntrl m_ballPosCntrl_y;

    float m_servo_commands[3]{};
    float m_reply_data[SPI_NUM_FLOATS]{};

    bool m_spi_ready{false};
    bool m_observerHasFirstMeasurement{false};

    // Userbutton
    DebounceIn user_button;
    bool m_executeMain{false};
    bool m_buttonCallbackAttached{false};

    void toggleExecuteMainFcn();

    void executeTask() override;

    static float clamp(float val, float min, float max);
    static float clamp01(float val) { return clamp(val, 0.0f, 1.0f); }

    static float DegreeToPWM(float degree, float range_degree);
    static float PWMToDegree(float pulse_width);
    static float DegreeToRad(float degree);
};

#endif /* SPI_COM_CNTRL_H_ */