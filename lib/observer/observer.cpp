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
    // --- P2, AUFGABE 2.3 ---
    // implement observer and apply time discrete integration step
    m_dxdt_hat = (m_A - m_H * m_C) * m_x_hat + m_B * u + m_H * y;
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

