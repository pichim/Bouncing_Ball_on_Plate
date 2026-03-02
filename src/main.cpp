#include "SPIComCntrl.h"
#include "config.h"
#include "mbed.h"

// TODOs:
// - Start with understanding this project

// static constexpr float SERVO_PULSE_MIN = 0.0325f;
// static constexpr float SERVO_PULSE_MAX = 0.1175f;

int main()
{
    SPIComCntrl spiComCntrl;
    spiComCntrl.enable();

    // Servo set to middle position
    // Servo servoD0(BBOP_SERVO_D0_PIN, BBOP_SERVO_PWM_PERIOD_US);
    // servoD0.calibratePulseMinMax(SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    // servoD0.enable(0.5f);

    while (true) {
        thread_sleep_for(1000);
    }
}
