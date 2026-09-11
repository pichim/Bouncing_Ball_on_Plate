# Bouncing Ball on Plate

Firmware for the Nucleo F446RE running a 1 kHz control loop with SPI-DMA slave I/O to a Raspberry Pi 5, MPU6500 IMU, three servos, and an optional high-speed UART logging stream.

Includes Python scripts for the vision pipeline.

## How to Use the System

### Wiring
1. **Servos:** Connect the laboratory cables to a DC power supply set to **7.4 V**.
2. **Nucleo:** Connect the Nucleo board to your PC or a power source using the black USB cable.
3. **Raspberry Pi:** Connect the white official power supply to the Raspberry Pi.

> **Important:** Keep the plate and base completely still for the first **5 seconds** after powering on the Nucleo so the IMU can calibrate its gyro/accel bias correctly.

### Start the Vision System

1. Log in to the Raspberry Pi via SSH or local terminal:
   - **User:** `pi`
   - **Password:** `Welcome123`
2. Run the startup script:

```bash
~/vision.sh
```

> **Note:** More details on how to start the vision scripts manually can be found [below](#run-on-the-raspberry-pi).


When the system is powered on, it requires approximately **5 seconds** to complete its initialization and calibration. During this time, **do not move the system or the plate**, as the IMU (Inertial Measurement Unit) is calibrating itself by measuring the plate's orientation. Once the calibration is complete, the system is ready for operation.

The operating modes are controlled using a single button. Each button press switches to the next mode:

1. **Home Position**
   The first button press moves the plate to its home position.

2. **Ball Balancing**
   Press the button again to start the ball balancing mode. The system automatically moves the ball to different setpoints every **6 seconds**.

3. **Circular Trajectory**
   Press the button once more to make the ball follow a circular trajectory on the plate.

4. **Bouncing Ball Mode**
   Press the button again to activate the bouncing ball mode. In this mode, the plate moves up and down to continuously bounce the ball while simultaneously moving it in a circular trajectory.

   To start this mode, drop the ball from approximately **10 cm above the plate**. The timing of the release is important, so it may take several attempts before the bouncing motion starts successfully. If it does not work on the first try, simply try again until the ball is synchronized with the plate's movement.

### Recommended Ball

For the best tracking performance, always use an **orange table tennis (ping pong) ball**.

## Hardware
 
- MCU: Nucleo-F446RE (Mbed)
- Master: Raspberry Pi 5 (SPI master)
- IMU: MPU6500 on I2C (PB_9/PB_8)
- Servos: PB_2, PC_8, PC_6 (20 ms PWM)
- SPI2 slave: MOSI PC_3, MISO PC_2, SCK PB_10, NSS PB_12 (DMA)
- UART log: PA_9 / PA_10 at 2 Mbps (SerialStream start-byte gated)
- external user button: PB_1
- Servo 1: D0/PB2
- Servo 2: D1/PC8
- Servo 3: D2/PC6
 
## Firmware layout
 
- src/main.cpp: boots and enables `SPIComCntrl` thread.
- lib/SPIComCntrl: 1 kHz realtime thread, bridges SPI frames to servo outputs and IMU telemetry.
- lib/SPISlaveDMA: DMA-based SPI slave with CRC-8 and double-transfer handshake (0x56 arm, 0x55 publish).
- lib/IMU: MPU6500 driver + filters + Mahony AHRS.
- lib/SerialStream: optional UART logger (start-byte triggered).
- lib/Servo: FastPWM-based servo driver (single-threaded, no mutex).
- include/config.h: pins, thread periods, filter settings, UART pins.
 
## Data link (SPI)
 
- Payload: 30 floats. First 3 = x, y, z.
- Reply fields populated: x, y, z (mm) (echo), gyro (rad/s), acc (m/s²) in first 9 floats; remainder zero.
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

### Option 1: Using the helper script

The helper script is located in the home directory. You can run it either by changing to the home directory:

```bash
./vision.sh
```

Or directly from anywhere:

```bash
~/vision.sh
```

---

### Option 2: Running manually

Navigate to the project directory:

```bash
cd ~/GIT_repositories/Bouncing_Ball_on_Plate
```

Run the application:

```bash
sudo chrt -f 50 venv/bin/python python/main.py
```

> **Note:** Calling `venv/bin/python` directly uses the virtual environment automatically without requiring `source venv/bin/activate` beforehand.

---

### Managing the Virtual Environment Manually

Navigate to the project root directory first:

```bash
cd ~/GIT_repositories/Bouncing_Ball_on_Plate
```

Activate the virtual environment:

```bash
source venv/bin/activate
```

Deactivate the virtual environment:

```bash
deactivate
```

## Dependencies

The system was developed and tested with the following package versions:

- `picamera2` (tested with `0.3.31`)
- `spidev` (tested with `3.5`)
- `opencv-python` (tested with `4.13.0.92`)
- `numpy` (tested with `1.24.4`)
- `Flask` (tested with `2.2.2`)