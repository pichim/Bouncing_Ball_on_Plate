#include "SPIComCntrl.h"
#include "config.h"
#include "mbed.h"

// TODOs:
// - Resolve warnings in main.py
// - Clean up project

int main()
{
    SPIComCntrl spiComCntrl;
    spiComCntrl.enable();

    while (true) {
        thread_sleep_for(1000);
    }
}
