#ifndef BBOP_CONFIG_H_
#define BBOP_CONFIG_H_

// SPI Communication with Raspberry Pi
#define BBOP_SPI_SLAVE_DMA_MOSI_PIN PC_3
#define BBOP_SPI_SLAVE_DMA_MISO_PIN PC_2
#define BBOP_SPI_SLAVE_DMA_SCK_PIN PB_10
#define BBOP_SPI_SLAVE_DMA_NSS_PIN PB_12

#define BBOP_SPI_SLAVE_DMA_THREAD_PRIORITY osPriorityHigh2
#define BBOP_SPI_SLAVE_DMA_THREAD_STACK_SIZE OS_STACK_SIZE

// PES Board IMU
#define BBOP_IMU_SDA_PIN PC_9
#define BBOP_IMU_SCL_PIN PA_8

// PES Board Servo Connections
#define BBOP_SERVO_D0_PIN PB_2
#define BBOP_SERVO_D1_PIN PC_8
#define BBOP_SERVO_D2_PIN PC_6

// SPI Communication and Control Thread
#define BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US 1000
#define BBOP_SPI_COM_CNTRL_THREAD_PRIORITY osPriorityNormal
#define BBOP_SPI_COM_CNTRL_THREAD_STACK_SIZE OS_STACK_SIZE

// // UART for logging communication (stream to PC for the fast running threads)
// #define BBOP_LOG_COM_UART_TX_PIN PG_14 // TX UART_6
// #define BBOP_LOG_COM_UART_RX_PIN PG_9  // RX UART_6

#endif /* BBOP_CONFIG_H_ */
