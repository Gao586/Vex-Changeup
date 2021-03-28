#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

/*vex-vision-config:begin*/
signature Vision__ZHUIZONG =
    signature(1, -3831, -3255, -3542, 8699, 9329, 9014, 11, 0);
signature Vision__SIG_2 = signature(2, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_3 = signature(3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_4 = signature(4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_5 = signature(5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_6 = signature(6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_7 = signature(7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision =
    vision(PORT21, 50, Vision__ZHUIZONG, Vision__SIG_2, Vision__SIG_3,
           Vision__SIG_4, Vision__SIG_5, Vision__SIG_6, Vision__SIG_7);
/*vex-vision-config:end*/
gyro Gyro = gyro(Brain.ThreeWirePort.A);
controller Controller1 = controller(controllerType::primary);
bumper BumperH = bumper(Brain.ThreeWirePort.H);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}
