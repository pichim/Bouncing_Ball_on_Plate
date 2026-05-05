#pragma once
#include <cmath>

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif

enum class TrajectoryMode
{
    Hold,
    Circle
};

struct TrajectoryRef
{
    float x_mm = 0.0f;
    float y_mm = 0.0f;

    float vx_mm_s = 0.0f;
    float vy_mm_s = 0.0f;

    float ax_mm_s2 = 0.0f;
    float ay_mm_s2 = 0.0f;
};

class TrajectoryGenerator
{
public:
    void setHold(float x_mm, float y_mm)
    {
        m_mode = TrajectoryMode::Hold;
        m_hold_x_mm = x_mm;
        m_hold_y_mm = y_mm;
    }

    void setCircle(float radius_mm, float freq_hz)
    {
        m_mode = TrajectoryMode::Circle;
        m_radius_mm = radius_mm;
        m_freq_hz = freq_hz;
    }

    void setMode(TrajectoryMode mode)
    {
        m_mode = mode;
    }

    TrajectoryMode getMode() const
    {
        return m_mode;
    }

    TrajectoryRef update(float t_s) const
    {
        TrajectoryRef ref;

        if (m_mode == TrajectoryMode::Hold)
        {
            ref.x_mm = m_hold_x_mm;
            ref.y_mm = m_hold_y_mm;

            ref.vx_mm_s = 0.0f;
            ref.vy_mm_s = 0.0f;

            ref.ax_mm_s2 = 0.0f;
            ref.ay_mm_s2 = 0.0f;
        }
        else if (m_mode == TrajectoryMode::Circle)
        {
            const float w = 2.0f * M_PIf * m_freq_hz;

            ref.x_mm = m_radius_mm * std::cos(w * t_s);
            ref.y_mm = m_radius_mm * std::sin(w * t_s);

            ref.vx_mm_s = -m_radius_mm * w * std::sin(w * t_s);
            ref.vy_mm_s =  m_radius_mm * w * std::cos(w * t_s);

            ref.ax_mm_s2 = -m_radius_mm * w * w * std::cos(w * t_s);
            ref.ay_mm_s2 = -m_radius_mm * w * w * std::sin(w * t_s);
        }

        return ref;
    }

private:
    TrajectoryMode m_mode = TrajectoryMode::Hold;

    float m_hold_x_mm = 0.0f;
    float m_hold_y_mm = 0.0f;

    float m_radius_mm = 20.0f;
    float m_freq_hz = 0.1f;
};