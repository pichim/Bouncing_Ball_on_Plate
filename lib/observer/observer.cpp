#include "observer.h"

// default constructor
observer::observer()
    : m_Ts(0.001f)
{
    init();
}

// constructor
observer::observer(float Ts)
    : m_Ts(Ts)
{
    init();
}

observer::~observer() {}

// calculate one step of the observer
Matrix<float, N, 1> observer::do_step(float u, float y)
{
    /*
     * Kamera-Totzeit:
     * nd = Tt / Ts = 0.016 / 0.001 = 16
     *
     * y_delayed ist der Messwert von vor 16 Observer-Schritten.
     * Am Anfang ist der Buffer mit 0 initialisiert.
     */

    float y_delayed = m_y_buffer[m_y_buffer_index];

    // aktuellen Messwert in Buffer schreiben
    m_y_buffer[m_y_buffer_index] = y;

    // Buffer-Index weiterschalten
    m_y_buffer_index++;

    if (m_y_buffer_index >= nd) {
        m_y_buffer_index = 0;
    }

    // Observer mit verzögertem Messwert
    m_dxdt_hat = (m_A - m_H * m_C) * m_x_hat + m_B * u + m_H * y_delayed;
    // m_dxdt_hat = (m_A - m_H * m_C) * m_x_hat + m_B * u + m_H * y;
    integrate_states();
    return m_x_hat;
}

// get the observed states
Matrix<float, N, 1> observer::get_x_obsv() { return m_x_hat; }

void observer::reset(float position_mm, float velocity_mm_s, float disturbance_rad)
{
    m_x_hat << position_mm,
               velocity_mm_s,
               disturbance_rad;

    m_dxdt_hat.setZero();

    m_y_buffer.fill(0.0f);
    m_y_buffer_index = 0;
}

float observer::getPositionMm() const
{
    return m_x_hat(0);
}

float observer::getVelocityMmS() const
{
    return m_x_hat(1);
}

float observer::getDisturbanceRad() const
{
    return m_x_hat(2);
}

void observer::init()
{
    // initialize all matrices with zeros
    m_A.setZero();
    m_B.setZero();
    m_C.setZero();
    m_H.setZero();
    m_dxdt_hat.setZero();
    m_x_hat.setZero();

    m_y_buffer.fill(0.0f);
    m_y_buffer_index = 0;

    // --- Matlab ---
    // set A, B, C, H matrices of observer
    m_A << 0.0f, 1.0f, 0.0f,
           0.0f, 0.0f, 5886.0f,
           0.0f, 0.0f, 0.0f;
    m_B << 0.0f, 5886.0f, 0.0f;
    m_C << 1.0f, 0.0f, 0.0f;
    m_H << 39.0980f, 714.3258f, 1.0000f;
}

void observer::integrate_states()
{
    // implement time discrete integration step
    m_x_hat += m_Ts * m_dxdt_hat;
}

