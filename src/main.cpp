#include "SPIComCntrl.h"
#include "config.h"
#include "mbed.h"
#include "InverseKinematics3Leg.h"

// TODOs:
// - Start with understanding this project

int main()
{
    // SPIComCntrl spiComCntrl;
    // spiComCntrl.enable();

    InverseKinematics3Leg ik;

    // Example usage of InverseKinematics3Leg
    InverseKinematics3Leg::Input ikInput;
    ikInput.roll = InverseKinematics3Leg::deg2rad(15);  // [rad] roll
    ikInput.pitch = InverseKinematics3Leg::deg2rad(0);  // [rad] pitch
    ikInput.h = 100.0;    // [mm] height

    InverseKinematics3Leg::Result ikResult = ik.compute(ikInput);
    
    if (ikResult.success) {
            printf("IK computation successful!\n");
            for (size_t i = 0; i < 3; ++i) {
                printf("Leg %zu: alpha = %.2f deg, rod error = %.2f mm\n",
                        i + 1, ikResult.alphaDeg[i], ikResult.rodError[i]);
            }
        } else {
            printf("IK computation failed: %s\n", ikResult.errorMessage.c_str());
        }    

    while (true) {
        thread_sleep_for(1000);
    }
}
