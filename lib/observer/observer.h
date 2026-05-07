#pragma once

#include <Eigen/Dense>
#include <cstdint>

#include "mbed.h"

#define N 2      // number of states
#define N_meas 1 // number measurements

using namespace Eigen;

class observer
{
public:
    observer() {};       // default constructor
    observer(float);     // constructor
    virtual ~observer(); // deconstructor

    Matrix<float, N, 1> do_step(float, float); // calculate one step of the observer
    Matrix<float, N, 1> get_x_obsv();          // get the observed states

private:
    float m_Ts;
    Matrix<float, N, N> m_A;
    Matrix<float, N, 1> m_B;
    Matrix<float, N_meas, N> m_C;
    Matrix<float, N, N_meas> m_H;
    Matrix<float, N, 1> m_dxdt_hat, m_x_hat;

    void init();
    void integrate_states();
};
