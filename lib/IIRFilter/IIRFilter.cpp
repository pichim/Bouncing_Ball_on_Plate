#include "IIRFilter.h"

#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265358979323846 // pi
#endif

// Integrator
// Time continous prototype: G(s) = 1 / s
// Disrectization method: Euler

void IIRFilter::integratorInit(const float Ts)
{
    filter.order = 1;
    integratorUpdate(Ts);
    reset(0.0f);
}

void IIRFilter::integratorUpdate(const float Ts)
{
    filter.B[0] = static_cast<double>(Ts);
    filter.B[1] = 0.0;
    filter.B[2] = 0.0;
    filter.A[0] = -1.0;
    filter.A[1] = 0.0;
}

// Differentiator
// Time continous prototype: G(s) = s
// Disrectization method: Euler

void IIRFilter::differentiatorInit(const float Ts)
{
    filter.order = 1;
    differentiatorUpdate(Ts);
    resetDifferentingFilterToZero(0.0f);
}

void IIRFilter::differentiatorUpdate(const float Ts)
{
    const double invTs = 1.0 / static_cast<double>(Ts);
    filter.B[0] = invTs;
    filter.B[1] = -invTs;
    filter.B[2] = 0.0;
    filter.A[0] = 0.0;
    filter.A[1] = 0.0;
}

// First Order Lowpass Filter
// Time continous prototype: G(s) = wcut / (s +  wcut)
// Disrectization method: ZOH with one additional forward shift, e.g. G(z^-1) = Gzoh(z^-1) * z

void IIRFilter::lowPass1Init(const float fcut, const float Ts)
{
    filter.order = 1;
    lowPass1Update(fcut, Ts);
    reset(0.0f);
}

void IIRFilter::lowPass1Update(const float fcut, const float Ts)
{
    filter.A[1] = 0.0;
    filter.B[0] = 1.0 - exp(-static_cast<double>(Ts) * 2.0 * M_PI * static_cast<double>(fcut));
    filter.B[1] = 0.0;
    filter.B[2] = 0.0;
    filter.A[0] = filter.B[0] - 1.0;
}

void IIRFilter::differentiatingLowPass1Init(const float fcut, const float Ts)
{
    filter.order = 1;
    differentiatingLowPass1Update(fcut, Ts);
    resetDifferentingFilterToZero(0.0f);
}

void IIRFilter::differentiatingLowPass1Update(const float fcut, const float Ts)
{
    const double b0 = 1.0 - exp(-static_cast<double>(Ts) * 2.0 * M_PI * static_cast<double>(fcut));
    filter.A[1] = 0.0;
    filter.B[0] = b0 / static_cast<double>(Ts);
    filter.B[1] = -filter.B[0];
    filter.B[2] = 0.0;
    filter.A[0] = b0 - 1.0;
}

// First Order Lead or Lag Filter
// Time continous prototype: G(s) = (wPole / wZero) * (s + wZero) / (s + wPole)
// Disrectization method: Tustin with prewarping

void IIRFilter::leadLag1Init(const float fZero, const float fPole, const float Ts)
{
    filter.order = 1;
    leadLag1Update(fZero, fPole, Ts);
    reset(0.0f);
}

void IIRFilter::leadLag1Update(const float fZero, const float fPole, const float Ts)
{
    const double wZero = (2.0 / static_cast<double>(Ts)) * tan(M_PI * static_cast<double>(fZero) * static_cast<double>(Ts));
    const double wPole = (2.0 / static_cast<double>(Ts)) * tan(M_PI * static_cast<double>(fPole) * static_cast<double>(Ts));
    const double k = 1.0 / (static_cast<double>(Ts) * wPole + 2.0);

    filter.B[0] = wPole * (static_cast<double>(Ts) * wZero + 2.0) / wZero * k;
    filter.B[1] = wPole * (static_cast<double>(Ts) * wZero - 2.0) / wZero * k;
    filter.B[2] = 0.0;
    filter.A[0] = (static_cast<double>(Ts) * wPole - 2.0) * k;
    filter.A[1] = 0.0;
}

void IIRFilter::phaseComp1Init(const float fCenter, const float phaseLift, const float Ts)
{
    filter.order = 1;
    phaseComp1Update(fCenter, phaseLift, Ts);
    reset(0.0f);
}

void IIRFilter::phaseComp1Update(const float fCenter, const float phaseLift, const float Ts)
{
    const double sn = sin(M_PI / 180.0 * static_cast<double>(phaseLift));
    const double k = sqrt((1.0 - sn) / (1.0 + sn));
    const double fZero = static_cast<double>(fCenter) * k;
    const double fPole = static_cast<double>(fCenter) / k;

    leadLag1Update(static_cast<float>(fZero), static_cast<float>(fPole), Ts);
}

// Second Order Notch Filter
// Time continous prototype: G(s) = (s^2 + wcut^2) / (s^2 + 2 * D * wcut * s + wcut^2)
// Disrectization method: Tustin with prewarping

void IIRFilter::notchInit(const float fcut, const float D, const float Ts)
{
    filter.order = 2;
    notchUpdate(fcut, D, Ts);
    reset(0.0f);
}

void IIRFilter::notchUpdate(const float fcut, const float D, const float Ts)
{
    // prewarp is done implicitly
    const double omega = 2.0 * M_PI * static_cast<double>(fcut) * static_cast<double>(Ts);
    const double sn = sin(omega);
    const double cs = cos(omega);

    filter.B[0] = 1.0 / (1.0 + static_cast<double>(D) * sn);
    filter.B[1] = -2.0 * cs * filter.B[0];
    filter.B[2] = filter.B[0];
    filter.A[0] = filter.B[1];
    filter.A[1] = (1.0 - static_cast<double>(D) * sn) * filter.B[0];
}

// Second Order Lowpass Filter
// Time continous prototype: G(s) = wcut^2 / (s^2 + 2 * D * wcut * s + wcut^2)
// Disrectization method: Euler

void IIRFilter::lowPass2Init(const float fcut, const float D, const float Ts)
{
    filter.order = 2;
    lowPass2Update(fcut, D, Ts);
    reset(0.0f);
}

void IIRFilter::lowPass2Update(const float fcut, const float D, const float Ts)
{
    const double wcut = 2.0 * M_PI * static_cast<double>(fcut);
    const double k1 = 2.0 * static_cast<double>(D) * static_cast<double>(Ts) * wcut;

    filter.A[1] = 1.0 / (static_cast<double>(Ts) * static_cast<double>(Ts) * wcut * wcut + k1 + 1.0);
    filter.B[0] = 1.0 - filter.A[1] * (1.0 + k1);
    filter.B[1] = 0.0;
    filter.B[2] = 0.0;
    filter.A[0] = filter.B[0] - 1.0 - filter.A[1];
}

// Second Order Lead or Lag Filter
// Time continous prototype: G(s) = (wPole^2 / wZero^2) * (s^2 + 2*DZero*wZero*s + wZero^2) / (s^2 + 2*DPole*wPole*s + wPole^2)
// Disrectization method: Tustin with prewarping

void IIRFilter::leadLag2Init(const float fZero, const float DZero, const float fPole, const float DPole, const float Ts)
{
    filter.order = 2;
    leadLag2Update(fZero, DZero, fPole, DPole, Ts);
    reset(0.0f);
}

void IIRFilter::leadLag2Update(const float fZero, const float DZero, const float fPole, const float DPole, const float Ts)
{
    // prewarp is done implicitly
    const double omegaZero = 2.0 * M_PI * static_cast<double>(fZero) * static_cast<double>(Ts);
    const double snZero = sin(omegaZero);
    const double csZero = cos(omegaZero);
    const double omegaPole = 2.0 * M_PI * static_cast<double>(fPole) * static_cast<double>(Ts);
    const double snPole = sin(omegaPole);
    const double csPole = cos(omegaPole);
    const double k0 = 1.0 / (1.0 + static_cast<double>(DPole) * snPole);
    const double k1 = k0 * (csPole - 1.0) / (csZero - 1.0);

    filter.B[0] = (1.0 + static_cast<double>(DZero) * snZero) * k1;
    filter.B[1] = -2.0 * csZero * k1;
    filter.B[2] = (1.0 - static_cast<double>(DZero) * snZero) * k1;
    filter.A[0] = -2.0 * csPole * k0;
    filter.A[1] = (1.0 - static_cast<double>(DPole) * snPole) * k0;
}

void IIRFilter::reset(const float output)
{
    // Choose internal state so that a constant input yields 'output'
    // as the steady-state output (given current A/B).
    filter.w[0] = static_cast<double>(output) * (1.0 - filter.B[0]);
    if (filter.order == 2) {
        filter.w[1] = filter.w[0] + static_cast<double>(output) * (filter.A[0] - filter.B[1]);
    }
}

// Assuming a constant input, differentiating results in zero output
// Currently only implemented for first order differentiators

void IIRFilter::resetDifferentingFilterToZero(const float output)
{
    filter.w[0] = static_cast<double>(output) * filter.B[1];
}

float IIRFilter::apply(const float input)
{
    const double output = filter.B[0] * static_cast<double>(input) + filter.w[0];
    applyFilterUpdate(static_cast<double>(input), output);
    return static_cast<float>(output);
}

float IIRFilter::applyConstrained(const float input, const float yMin, const float yMax)
{
    // constrain output
    const double outputUnconstrained = filter.B[0] * static_cast<double>(input) + filter.w[0];
    const double output = (outputUnconstrained < static_cast<double>(yMin)) ? static_cast<double>(yMin)
                        : (outputUnconstrained > static_cast<double>(yMax)) ? static_cast<double>(yMax)
                        :  outputUnconstrained;
    applyFilterUpdate(static_cast<double>(input), output);
    return static_cast<float>(output);
}

void IIRFilter::applyFilterUpdate(const double input, const double output)
{
    // https://dsp.stackexchange.com/questions/72575/transposed-direct-form-ii
    for (unsigned i = 0; i < filter.order - 1; ++i) {
        filter.w[i] = filter.B[i + 1] * input + filter.w[i + 1] - filter.A[i] * output;
    }
    filter.w[filter.order - 1] = filter.B[filter.order] * input - filter.A[filter.order - 1] * output;
}
