#include "KalmanBall1D.h"

#include <cmath>

KalmanBall1D::KalmanBall1D()
    : m_TsPredict(0.001f)
    , m_TsCamera(0.020f)
    , m_B(0.0f)
    , m_initialized(false)
    , m_x(Eigen::Vector3f::Zero())
    , m_L(Eigen::Vector3f::Zero())
    , m_maxInnovationMm(100.0f)
    , m_maxDisturbanceRad(0.15f)
{
}

void KalmanBall1D::init(float Ts_predict_s,
                        float Ts_camera_s,
                        float g_mm_s2,
                        const Eigen::Vector3f& He_continuous)
{
    m_TsPredict = Ts_predict_s;
    m_TsCamera  = Ts_camera_s;

    /*
     * Ball model:
     *
     * x_ddot = 3/5 * g * theta
     */
    m_B = (3.0f / 5.0f) * g_mm_s2;

    /*
     * He from MATLAB is a continuous observer gain.
     *
     * In C++ we only correct when a new camera measurement arrives.
     * Therefore we approximate the discrete correction gain with:
     *
     * L = He * Ts_camera
     */
    m_L = He_continuous * m_TsCamera;

    reset();
}

void KalmanBall1D::reset(float position_mm,
                         float velocity_mm_s,
                         float disturbance_rad)
{
    m_x(0) = position_mm;
    m_x(1) = velocity_mm_s;
    m_x(2) = disturbance_rad;

    m_initialized = true;
}

void KalmanBall1D::predict(float plate_angle_rad)
{
    if (!m_initialized) {
        reset();
    }

    /*
     * State:
     *
     * x[0] = ball position     [mm]
     * x[1] = ball velocity     [mm/s]
     * x[2] = disturbance angle [rad]
     *
     * Model:
     *
     * x_ddot = B * (plate_angle_rad + disturbance_rad)
     */

    const float Ts  = m_TsPredict;
    const float Ts2 = Ts * Ts;

    const float position_mm      = m_x(0);
    const float velocity_mm_s    = m_x(1);
    const float disturbance_rad  = m_x(2);

    const float acceleration_mm_s2 =
        m_B * (plate_angle_rad + disturbance_rad);

    m_x(0) = position_mm
           + Ts * velocity_mm_s
           + 0.5f * Ts2 * acceleration_mm_s2;

    m_x(1) = velocity_mm_s
           + Ts * acceleration_mm_s2;

    /*
     * Disturbance is modeled as constant.
     */
    m_x(2) = disturbance_rad;

    limitDisturbance();
}

bool KalmanBall1D::update(float measured_position_mm)
{
    if (!std::isfinite(measured_position_mm)) {
        return false;
    }

    if (!m_initialized) {
        reset(measured_position_mm, 0.0f, 0.0f);
        return true;
    }

    /*
     * Camera measures only the ball position:
     *
     * y = x_position
     */
    const float innovation_mm = measured_position_mm - m_x(0);

    /*
     * Safety check:
     * If the camera detection jumps too much, ignore this measurement.
     */
    if (std::fabs(innovation_mm) > m_maxInnovationMm) {
        return false;
    }

    /*
     * Static Kalman observer correction:
     *
     * x_hat = x_hat + L * innovation
     */
    m_x += m_L * innovation_mm;

    limitDisturbance();

    return true;
}

float KalmanBall1D::getPositionMm() const
{
    return m_x(0);
}

float KalmanBall1D::getVelocityMmS() const
{
    return m_x(1);
}

float KalmanBall1D::getDisturbanceRad() const
{
    return m_x(2);
}

Eigen::Vector3f KalmanBall1D::getState() const
{
    return m_x;
}

void KalmanBall1D::setMaxInnovationMm(float maxInnovationMm)
{
    m_maxInnovationMm = maxInnovationMm;
}

void KalmanBall1D::setMaxDisturbanceRad(float maxDisturbanceRad)
{
    m_maxDisturbanceRad = maxDisturbanceRad;
}

void KalmanBall1D::limitDisturbance()
{
    m_x(2) = clampFloat(m_x(2),
                        -m_maxDisturbanceRad,
                         m_maxDisturbanceRad);
}

float KalmanBall1D::clampFloat(float value,
                               float minValue,
                               float maxValue) const
{
    if (value < minValue) {
        return minValue;
    }

    if (value > maxValue) {
        return maxValue;
    }

    return value;
}