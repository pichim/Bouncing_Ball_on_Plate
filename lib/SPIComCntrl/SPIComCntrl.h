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
#include "KalmanBall1D.h"
#include "DebounceIn.h"
#include "TrajectoryGenerator.h"

using namespace std::chrono;

class SPIComCntrl : public RealTimeThread
{
public:
    explicit SPIComCntrl();
    virtual ~SPIComCntrl();

private:
    //powerhd 1
    static constexpr float SERVO1_PULSE_MIN = 0.3f;
    static constexpr float SERVO1_PULSE_MAX = 0.695f;  

    //powerhd 2
    static constexpr float SERVO2_PULSE_MIN = 0.305f;
    static constexpr float SERVO2_PULSE_MAX = 0.7025f;  

    //powerhd 3
    static constexpr float SERVO3_PULSE_MIN = 0.3025f;
    static constexpr float SERVO3_PULSE_MAX = 0.6975f; 

    static constexpr float BALL_POS_CENTER_PX = 0.0f;
    static constexpr float ANGLE_DELTA_LIMIT_GRAD = 20.0f;
    
    // // Michis Reglerwerte
    // static constexpr float BALL_CTRL_KP = 0.1f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_D_S = 0.0796f; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0265f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (0.4157f - 0.0413f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    // // Ximus Reglerwerte
    // static constexpr float BALL_CTRL_KP = 0.22061;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_V = 2.1519f; 
    // static constexpr float BALL_CTRL_TAU_f = 0.1162f;  
    // static constexpr float BALL_CTRL_TAU_R_O = 0.039789f * 5.0f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    // // Luca Reglerwerte
    // static constexpr float BALL_CTRL_KP = 0.0579f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_V = 4.5f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_f = 0.2217f * 1.0f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;    
    // static constexpr float BALL_CTRL_TAU_R_O = 0.1314f * 5.0f;//0.03f * 2.0f; //0.0265f * 5.0f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

   // michi Zwischenstand 
    // static constexpr float BALL_CTRL_KP = 0.2383 * 0.3f; //0.2f;
    // static constexpr float BALL_CTRL_KI = 0.0f;
    // static constexpr float BALL_CTRL_TAU_V = 4.5f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;
    // static constexpr float BALL_CTRL_TAU_f = 0.1314f * 2.5f;//0.06f * 1.0f;//0.1314f * 0.1; //0.0413f * 1.0f;    
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0398f * 5.0f;//0.03f * 2.0f; //0.0265f * 5.0f; //0.0138f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds


    // alter kalman mit ff etc.
    // static constexpr float BALL_CTRL_KP = 0.04f;
    // static constexpr float BALL_CTRL_KI = 0.005f;
    // static constexpr float BALL_CTRL_TAU_V = 1.2f;
    // static constexpr float BALL_CTRL_TAU_f = 0.6f;
    // static constexpr float BALL_CTRL_TAU_R_O = 0.0398f * 5.0f;
    // static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    // static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    // 333 hz ohne kalman
    static constexpr float BALL_CTRL_KP = 0.05f;
    static constexpr float BALL_CTRL_KI = 0.004f;
    static constexpr float BALL_CTRL_TAU_V = 0.9f;
    static constexpr float BALL_CTRL_TAU_f = 0.1f;
    static constexpr float BALL_CTRL_TAU_R_O = 0.01f;
    static constexpr float BALL_CTRL_KD = BALL_CTRL_KP * (BALL_CTRL_TAU_V - BALL_CTRL_TAU_f);
    static constexpr float VISION_TIMEOUT = 1.0F; // seconds

    
    static constexpr float PI = 3.14159265358979323846f;

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

    InverseKinematics3Leg m_ik;
    InverseKinematics3Leg::Input m_ikInput;

    KalmanBall1D m_kalmanX;
    KalmanBall1D m_kalmanY;

    bool m_kalmanHasFirstMeasurement = false;

    TrajectoryGenerator m_trajectory;


    float m_Ts;
    PIDCntrl m_ballPosCntrl_x;
    PIDCntrl m_ballPosCntrl_y;

    float m_servo_commands[3]{};

    float m_reply_data[SPI_NUM_FLOATS];

    bool m_spi_ready{false};

    //userbutton
    DebounceIn user_button;
    bool m_executeMain{false};
    void toggleExecuteMainFcn();

    void executeTask() override;
    static float clamp(float val, float min, float max);
    static float clamp01(float val) { return clamp(val, 0.0f, 1.0f); }
    static float DegreeToPWM(float degree, float range_degree);
    static float PWMToDegree(float pulse_width);
    static float DegreeToRad(float degree);
};
#endif /* SPI_COM_CNTRL_H_ */