#include "SPIComCntrl.h"
#include "config.h"
#include "mbed.h"

// TODOs:
// - Use proper PWM (FastPWM for the servo control)
// - Include IMU MPU 6500 from MiniSegway (I2C)
// - Integrate SerialStream for data logging and debugging
// - Clean up project

int main()
{
    SPIComCntrl spiComCntrl(BBOP_SPI_SLAVE_DMA_MOSI_PIN,
                            BBOP_SPI_SLAVE_DMA_MISO_PIN,
                            BBOP_SPI_SLAVE_DMA_SCK_PIN,
                            BBOP_SPI_SLAVE_DMA_NSS_PIN,
                            BBOP_SPI_SLAVE_DMA_THREAD_PRIORITY,
                            BBOP_SPI_SLAVE_DMA_THREAD_STACK_SIZE,
                            BBOP_IMU_SDA_PIN,
                            BBOP_IMU_SCL_PIN,
                            BBOP_SERVO_D0_PIN,
                            BBOP_SERVO_D1_PIN,
                            BBOP_SERVO_D2_PIN,
                            BBOP_SPI_COM_CNTRL_THREAD_PERIOD_US,
                            BBOP_SPI_COM_CNTRL_THREAD_PRIORITY,
                            BBOP_SPI_COM_CNTRL_THREAD_STACK_SIZE);
    spiComCntrl.enable();

    while (true) {
        thread_sleep_for(1000);
    }
}
