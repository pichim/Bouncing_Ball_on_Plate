#ifndef SERVO_H_
#define SERVO_H_

#include "FastPWM.h"
#include "mbed.h"

/**
 * @brief Class for smooth control of a servo motor.
 *
 * This class provides functionalities to control servo motors with
 * features like calibration, setting motion profiles, and enabling/disabling the servo.
 */
class Servo
{
public:
    /**
     * @brief Construct a new Servo object.
     *
     * @param pin PWM-capable pin connected to the servo.
     * @param period_us PWM period in microseconds (e.g., 20000 for 50 Hz servos).
     */
    explicit Servo(PinName pin, int period_us);

    /**
     * @brief Destroy the Servo object.
     */
    virtual ~Servo();

    /**
     * @brief Calibrate the minimum and maximum pulse widths for the servo.
     *
     * @param pulse_min The minimum pulse width.
     * @param pulse_max The maximum pulse width.
     */
    void calibratePulseMinMax(float pulse_min = 0.0f, float pulse_max = 1.0f);

    /**
     * @brief Set the normalised pulse width.
     *
     * @param pulse The pulse width to be set, normalised.
     */
    void setPulseWidth(float pulse = 0.0f);

    /**
     * @brief Enable the servo with a specific pulse width.
     *
     * @param pulse The pulse width to set when enabling.
     */
    void enable(float pulse = 0.0f);

    /**
     * @brief Disable the servo.
     */
    void disable();

    /**
     * @brief Check if the servo is enabled.
     *
     * @return true If the servo is enabled.
     * @return false If the servo is disabled.
     */
    bool isEnabled() const;

private:
    static constexpr float PWM_MIN = 0.01f;
    static constexpr float PWM_MAX = 0.99f;

    FastPWM m_pwm;

    int m_period_us;

    bool m_enabled{false};
    float m_pulse{0.0f};
    float m_pulse_min{0.0f};
    float m_pulse_max{1.0f};

    float calculateNormalisedPulseWidth(float pulse);
    float constrainPulse(float pulse) const;
};

#endif /* SERVO_H_ */
