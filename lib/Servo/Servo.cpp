#include "Servo.h"

Servo::Servo(PinName pin, int period_us)
    : m_pwm(pin)
    , m_period_us(period_us)
{
    m_pwm.period_mus(m_period_us);
}

Servo::~Servo() { disable(); }

void Servo::calibratePulseMinMax(float pulse_min, float pulse_max)
{
    // set minimal and maximal pulse width
    m_pulse_min = pulse_min;
    m_pulse_max = pulse_max;
}

void Servo::setPulseWidth(float pulse)
{
    // after calibrating the mapping from setPulseWidth() is (0, 1) -> (pulse_min, pulse_max)
    m_pulse = calculateNormalisedPulseWidth(pulse);

    if (m_enabled) {
        const uint16_t pulse_mus = static_cast<uint16_t>(m_pulse * static_cast<float>(m_period_us));
        m_pwm.pulsewidth_us(pulse_mus);
    }
}

void Servo::enable(float pulse)
{
    m_enabled = true;
    m_pulse = calculateNormalisedPulseWidth(pulse);

    const uint16_t pulse_mus = static_cast<uint16_t>(m_pulse * static_cast<float>(m_period_us));
    m_pwm.pulsewidth_us(pulse_mus);
}

void Servo::disable()
{
    m_enabled = false;

    // drive PWM low
    m_pwm.pulsewidth_us(0);
}

bool Servo::isEnabled() const
{
    return m_enabled;
}

float Servo::calculateNormalisedPulseWidth(float pulse)
{
    // it is assumed that after the calibration m_pulse_min != 0.0f and if so
    // we constrain the pulse to the range (0.0f, 1.0f)
    if (m_pulse_min != 0.0f)
        pulse = (pulse > 1.0f) ? 1.0f : (pulse < 0.0f) ? 0.0f : pulse;
    return constrainPulse((m_pulse_max - m_pulse_min) * pulse + m_pulse_min);
}

float Servo::constrainPulse(float pulse) const
{
    // constrain pulse to range (PWM_MIN, PWM_MAX)
    return (pulse > PWM_MAX) ? PWM_MAX : (pulse < PWM_MIN) ? PWM_MIN : pulse;
}
