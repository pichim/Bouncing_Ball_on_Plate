# Bouncing Ball on Plate

Firmware for the Nucleo F446RE running a 1 kHz control loop with SPI-DMA slave I/O to a Raspberry Pi 5, MPU6500 IMU, three servos, and an optional high-speed UART logging stream.

## Hardware

- MCU: Nucleo-F446RE (Mbed)
- Master: Raspberry Pi 5 (SPI master)
- IMU: MPU6500 on I2C (PB_9/PB_8)
- Servos: PB_2, PC_8, PC_6 (20 ms PWM)
- SPI2 slave: MOSI PC_3, MISO PC_2, SCK PB_10, NSS PB_12 (DMA)
- UART log: PA_9 / PA_10 at 2 Mbps (SerialStream start-byte gated)

## Firmware layout

- src/main.cpp: boots and enables `SPIComCntrl` thread.
- lib/SPIComCntrl: 1 kHz realtime thread, bridges SPI frames to servo outputs and IMU telemetry.
- lib/SPISlaveDMA: DMA-based SPI slave with CRC-8 and double-transfer handshake (0x56 arm, 0x55 publish).
- lib/IMU: MPU6500 driver + filters + Mahony AHRS.
- lib/SerialStream: optional UART logger (start-byte triggered).
- lib/Servo: FastPWM-based servo driver (single-threaded, no mutex).
- include/config.h: pins, thread periods, filter settings, UART pins.

## Data link (SPI)

- Payload: 30 floats. First 3 = servo setpoints in [0,1].
- Reply fields populated: setpoints, gyro (rad/s), acc (m/s²), RPY (rad) in first 13 floats; remainder zero.
- Protocol: Master sends 0x56 (arm) then 0x55 (publish). CRC-8 (poly 0x07) over header+payload.

## Logging (UART)

- SerialStream transmits only after receiving start byte 255. Sends one byte with float-count, then the floats.

## Build & flash

- Tooling: PlatformIO, untested with Mbed Studio.

## Defaults / tuning

- Loop period: 1 ms (`BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US`).
- IMU filters: 60 Hz gyro/acc; 1000-sample skip, 1000-sample bias avg; optional static acc bias.
- Servo pulse bounds: 0.0325–0.1175 (normalized mapping).
- SPI payload length and UART logging buffer both capped at 30 floats.

## Notes

- Change pins and timing in `include/config.h` to retarget hardware.
- If IMU scale factors look zeroed, verify MPU6500 WHO_AM_I and I2C wiring.

## Run on the Raspberry Pi

Run: `sudo chrt -f 50 python python/main.py`
