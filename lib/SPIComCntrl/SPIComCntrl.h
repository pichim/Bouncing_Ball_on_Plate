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
#include <cmath>

using namespace std::chrono;

class SPIComCntrl : public RealTimeThread
{
public:
    explicit SPIComCntrl();
    virtual ~SPIComCntrl();

private:
    // MKS
    static constexpr float SERVO_PULSE_MIN = 0.214f;
    static constexpr float SERVO_PULSE_MAX = 0.536f;
    // reely S0090
    // float SERVO_PULSE_MIN = 0.0325f;
    // float SERVO_PULSE_MAX = 0.1175f;
    static constexpr float BALL_POS_CENTER_PX = 0.0f;
    static constexpr float SERVO_CENTER = 0.5161f;
    static constexpr float ANGLE_DELTA_LIMIT_GRAD = 30.0f;
    // Michis Reglerwerte
    // static constexpr float BALL_CTRL_KP = 0.1f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_D_S = 0.0796f; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0265f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (0.4157f - 0.0413f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    // static constexpr float BALL_CTRL_KP = 0.003f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_D_S = 0.0531f; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0177f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (0.4157f - 0.0413f) * 4;
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

// läuft
    // static constexpr float BALL_CTRL_KP = 0.03f; //0.0586f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_V = 4.0f;//2.643f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_f = 0.2627f * 1.0f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;    
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0318f * 5.0f;//0.03f * 2.0f; //0.0265f * 5.0f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    // static constexpr float BALL_CTRL_KP = 0.08f; //0.0586f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.001f;
    // static constexpr float BALL_CTRL_TAU_V = 6.0f;//2.643f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_f = 0.2627f * 1.5f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;    
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0318f * 5.0f;//0.03f * 2.0f; //0.0265f * 5.0f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    // good but still too fast
    // static constexpr float BALL_CTRL_KP = 0.2383 * 0.5f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_V = 4.0f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_f = 0.1314f * 3.0f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;    
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0398f * 3.0f;//0.03f * 2.0f; //0.0265f * 5.0f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    static constexpr float BALL_CTRL_KP = 0.2383 * 0.3f; //0.2f;
    static constexpr float BALL_CTRL_KI = 0.0f;
    static constexpr float BALL_CTRL_TAU_V = 3.0f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;
    static constexpr float BALL_CTRL_TAU_f = 0.1314f * 2.0f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;    
    static constexpr float BALL_CTRL_TAU_R_O = 0.0398f * 5.0f;//0.03f * 2.0f; //0.0265f * 5.0f; //0.0138f;
    static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    float cos_theta_rotation;
    float sin_theta_rotation;
    float camera_offset_angle_deg;
    float camera_offset_angle_rad;

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
    PIDCntrl m_ballPosCntrl_x;
    PIDCntrl m_ballPosCntrl_y;

    //userbutton
    DebounceIn user_button;

    float m_servo_commands[3]{};

    float m_reply_data[SPI_NUM_FLOATS];

    bool m_spi_ready{false};
    bool m_executeMain{false};

    void executeTask() override;
    static float clamp(float val, float min, float max);
    static float clamp01(float val) { return clamp(val, 0.0f, 1.0f); }
    static float DegreeToPWM(float degree);
    static float PWMToDegree(float pulse_width);
    static float DegreeToRad(float degree);

private:
    void toggleExecuteMainFcn();
};
#endif /* SPI_COM_CNTRL_H_ */
