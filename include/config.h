#ifndef BBOP_CONFIG_H_
#define BBOP_CONFIG_H_

// SPI Communication with Raspberry Pi
#define BBOP_SPI_SLAVE_DMA_MOSI_PIN PC_3
#define BBOP_SPI_SLAVE_DMA_MISO_PIN PC_2
#define BBOP_SPI_SLAVE_DMA_SCK_PIN PB_10
#define BBOP_SPI_SLAVE_DMA_NSS_PIN PB_12

#define BBOP_SPI_SLAVE_DMA_THREAD_PRIORITY osPriorityHigh2
#define BBOP_SPI_SLAVE_DMA_THREAD_STACK_SIZE OS_STACK_SIZE

// MPU6500 IMU Configuration
#define BBOP_IMU_SDA_PIN PB_9 // would be PC_9 on new PES_Board
#define BBOP_IMU_SCL_PIN PB_8 // would be PA_8 on new PES_Board
#define BBOP_IMU_USE_ADDITIONAL_FILTERS true
#define BBOP_IMU_GYRO_FILTER_FREQUENCY_HZ 60.0f
#define BBOP_IMU_ACC_FILTER_FREQUENCY_HZ 60.0f
#define BBOP_IMU_NUM_RUNS_SKIP 1000 // dont make this shorter than 1000 milliseconds, the openlager needs 1 second to start up (if used for logging)
#define BBOP_IMU_NUM_RUNS_FOR_AVERAGE 1000
#define BBOP_IMU_DO_USE_STATIC_ACC_CALIBRATION true // if this is true then averaged acc gets overwritten by BBOP_IMU_B_ACC
#define BBOP_IMU_B_ACC {0.0f, 0.0f, 0.0f}
#define BBOP_IMU_KP (0.1592f * 2.0f * M_PIf)
#define BBOP_IMU_KI 0.0f

// PES Board Servo Connections
#define BBOP_SERVO_D0_PIN PB_2
#define BBOP_SERVO_D1_PIN PC_8
#define BBOP_SERVO_D2_PIN PC_6
#define BBOP_SERVO_PWM_PERIOD_US 20000

// SPI Communication and Control Thread
#define BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US 1000
#define BBOP_SPI_COM_CNTRL_THREAD_PRIORITY osPriorityNormal
#define BBOP_SPI_COM_CNTRL_THREAD_STACK_SIZE OS_STACK_SIZE

// UART for logging communication (stream to PC for the fast running threads)
#define BBOP_LOG_COM_UART_TX_PIN PA_9
#define BBOP_LOG_COM_UART_RX_PIN PA_10

#endif /* BBOP_CONFIG_H_ */
