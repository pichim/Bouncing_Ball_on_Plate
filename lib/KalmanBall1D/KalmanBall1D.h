#pragma once

#include <Eigen/Dense>

class KalmanBall1D
{
public:
    KalmanBall1D();

    void init(float Ts_predict_s,
              float Ts_camera_s,
              float g_mm_s2,
              const Eigen::Vector3f& He_continuous);

    void reset(float position_mm = 0.0f,
               float velocity_mm_s = 0.0f,
               float disturbance_rad = 0.0f);

    void predict(float plate_angle_rad);

    bool update(float measured_position_mm);

    float getPositionMm() const;
    float getVelocityMmS() const;
    float getDisturbanceRad() const;

    Eigen::Vector3f getState() const;

    void setMaxInnovationMm(float maxInnovationMm);
    void setMaxDisturbanceRad(float maxDisturbanceRad);

private:
    void limitDisturbance();
    float clampFloat(float value, float minValue, float maxValue) const;

private:
    float m_TsPredict;
    float m_TsCamera;

    float m_B;

    bool m_initialized;

    Eigen::Vector3f m_x;
    Eigen::Vector3f m_L;

    float m_maxInnovationMm;
    float m_maxDisturbanceRad;
};