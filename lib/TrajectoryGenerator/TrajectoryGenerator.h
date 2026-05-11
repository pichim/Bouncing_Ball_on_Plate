#pragma once
#include <cmath>
#include <cstdint>

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif

static constexpr int TRAJ_MAX_SEQUENCE_STEPS = 16;

enum class TrajectoryMode
{
    Hold,
    Circle,
    FigureEight,
    Sequence,
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

struct SequencePoint
{
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float dwell_s = 3.0f; // Sekunden
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
        m_t_s = 0.0f;
    }

    void setFigureEight(float radius_x_mm, float radius_y_mm, float freq_hz)
    {
        m_mode = TrajectoryMode::FigureEight;
        m_fig8_radius_x_mm = radius_x_mm;
        m_fig8_radius_y_mm = radius_y_mm;
        m_fig8_freq_hz = freq_hz;
        m_t_s = 0.0f;
    }

    bool setSequence(const SequencePoint* points, int count)
    {
        if (count <= 0 || count > TRAJ_MAX_SEQUENCE_STEPS)
            return false;

        for (int i = 0; i < count; ++i)
            m_sequence[i] = points[i];

        m_sequence_count = count;
        m_seq_index      = 0;
        m_seq_elapsed_s  = 0.0f;
        m_mode           = TrajectoryMode::Sequence;
        return true;
    }

    void setMode(TrajectoryMode mode)
    {
        m_mode = mode;
    }

    TrajectoryMode getMode() const
    {
        return m_mode;
    }

    int getSequenceIndex() const
    {
        return m_seq_index;
    }

    TrajectoryRef update(float dt_s)
    {
        TrajectoryRef ref;

        switch (m_mode)
        {
            case TrajectoryMode::Hold:
            {
                ref.x_mm = m_hold_x_mm;
                ref.y_mm = m_hold_y_mm;
                break;
            }

            case TrajectoryMode::Circle:
            {
                m_t_s += dt_s;
                const float w = 2.0f * M_PIf * m_freq_hz;

                ref.x_mm = m_radius_mm * std::cos(w * m_t_s);
                ref.y_mm = m_radius_mm * std::sin(w * m_t_s);

                ref.vx_mm_s = -m_radius_mm * w * std::sin(w * m_t_s);
                ref.vy_mm_s =  m_radius_mm * w * std::cos(w * m_t_s);

                ref.ax_mm_s2 = -m_radius_mm * w * w * std::cos(w * m_t_s);
                ref.ay_mm_s2 = -m_radius_mm * w * w * std::sin(w * m_t_s);
                break;
            }

            case TrajectoryMode::FigureEight:
            {
                m_t_s += dt_s;
                const float w = 2.0f * M_PIf * m_fig8_freq_hz;

                const float wt = w * m_t_s;

                // Liegende 8
                ref.x_mm = m_fig8_radius_x_mm * std::sin(wt);
                ref.y_mm = m_fig8_radius_y_mm * std::sin(2.0f * wt);

                // Geschwindigkeit
                ref.vx_mm_s = m_fig8_radius_x_mm * w * std::cos(wt);
                ref.vy_mm_s = m_fig8_radius_y_mm * 2.0f * w * std::cos(2.0f * wt);

                // Beschleunigung
                ref.ax_mm_s2 = -m_fig8_radius_x_mm * w * w * std::sin(wt);
                ref.ay_mm_s2 = -m_fig8_radius_y_mm * 4.0f * w * w * std::sin(2.0f * wt);

                break;
            }

            case TrajectoryMode::Sequence:
            {
                if (m_sequence_count == 0)
                    break;

                m_seq_elapsed_s += dt_s;
                if (m_seq_elapsed_s >= m_sequence[m_seq_index].dwell_s)
                {
                    m_seq_elapsed_s = 0.0f;
                    m_seq_index = (m_seq_index + 1) % m_sequence_count;
                }

                ref.x_mm = m_sequence[m_seq_index].x_mm;
                ref.y_mm = m_sequence[m_seq_index].y_mm;
                break;
            }
        }

        return ref;
    }

private:
    TrajectoryMode m_mode = TrajectoryMode::Hold;

    // Hold
    float m_hold_x_mm = 0.0f;
    float m_hold_y_mm = 0.0f;

    // Circle
    float m_radius_mm = 20.0f;
    float m_freq_hz   = 0.1f;

    // Figure Eight
    float m_fig8_radius_x_mm = 30.0f;
    float m_fig8_radius_y_mm = 20.0f;
    float m_fig8_freq_hz     = 0.1f;

    // Common time for circle and figure eight
    float m_t_s = 0.0f;

    // Sequence
    SequencePoint m_sequence[TRAJ_MAX_SEQUENCE_STEPS] = {};
    int   m_sequence_count  = 0;
    int   m_seq_index       = 0;
    float m_seq_elapsed_s   = 0.0f;
};