/*
* Test program for servo control using PID/PD controller.
* 
*
*/

#include "PIDCntrl.h"
#include "Servo.h"
#include "config.h"
#include "mbed.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>

using namespace std::chrono_literals;

static BufferedSerial pc(BBOP_LOG_COM_UART_TX_PIN, BBOP_LOG_COM_UART_RX_PIN, 115200);

static void pc_printf(const char *fmt, ...) {
	char buf[256];
	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (n > 0) {
		pc.write(buf, n);
	}
}

static float fake_ball_position(float t_s) {
	constexpr float amplitude_m = 0.04f;
	constexpr float frequency_hz = 0.2f;
	constexpr float omega = 2.0f * 3.14159265358979323846f * frequency_hz;
	return amplitude_m * sinf(omega * t_s);
}

int main() {
	constexpr bool use_pid = false;

	constexpr float Ts_s = 1.0f / 60.0f;
	const auto Ts_tick = std::chrono::duration_cast<Kernel::Clock::duration>(std::chrono::duration<float>(Ts_s));
	constexpr float x_ref_m = 0.00f;

	constexpr float kp = 4.0f;
	constexpr float ki = 1.2f;
	constexpr float kd = 0.18f;
	constexpr float tau_d_s = 0.03f;

	constexpr float servo_center = 0.50f;
	constexpr float servo_delta_limit = 0.45f;

	Servo servo(BBOP_SERVO_D0_PIN, BBOP_SERVO_PWM_PERIOD_US);
	servo.calibratePulseMinMax(0.0325f, 0.1175f);
	servo.enable(servo_center);

	PIDCntrl controller;
	controller.setup(kp, use_pid ? ki : 0.0f, kd, tau_d_s, Ts_s, -servo_delta_limit, servo_delta_limit);
	controller.setIntegratorLimits(-0.25f, 0.25f);
	controller.reset(0.0f);

	pc_printf("\n=== Servo Ball Control Test ===\n");
	pc_printf("Controller: %s | Ts=%.3f s\n", use_pid ? "PID" : "PD", Ts_s);

	Timer t;
	t.start();

	uint32_t log_div = 0;

	while (true) {
		const float time_s = std::chrono::duration<float>(t.elapsed_time()).count();
		const float x_meas_m = fake_ball_position(time_s);
		const float error_m = x_ref_m - x_meas_m;

		const float servo_delta = controller.update(error_m, x_meas_m);
		const float servo_raw = servo_center + servo_delta;
		const float servo_cmd = (servo_raw > 1.0f) ? 1.0f : (servo_raw < 0.0f) ? 0.0f : servo_raw;
		servo.setPulseWidth(servo_cmd);

		if ((log_div++ % 10u) == 0u) {
			pc_printf("t=%.2f  x=%.4f  e=%.4f  du=%.4f  u=%.4f\n",
					  time_s,
					  x_meas_m,
					  error_m,
					  servo_delta,
					  servo_cmd);
		}

		ThisThread::sleep_for(Ts_tick);
	}
}
