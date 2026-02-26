#include "SPIComCntrl.h"
#include "config.h"
#include "mbed.h"

// TODOs:
// - Start with understanding this project

int main()
{
    SPIComCntrl spiComCntrl;
    spiComCntrl.enable();

    while (true) {
        thread_sleep_for(1000);
    }
}
