#include "motorControl.h"
#include "vex.h"
using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;
// A global instance of vex::competition
vex::competition Competition;

// define your global instances of motors and other devices here
motor LeftChassis2 = motor(LeftChassis2Port, ratio18_1, true);
motor LeftChassis1 = motor(LeftChassis1Port, ratio18_1, false);
motor RightChassis2 = motor(RightChassis2Port, ratio18_1, false);
motor RightChassis1 = motor(RightChassis1Port, ratio18_1, true);
motor FrontRollers = motor(FrontRollersPort, ratio6_1, true);
motor LaunchRoller = motor(LaunchRollerPort, ratio18_1, true);
motor LeftIntake1 = motor(LeftIntake1Port, ratio6_1, true);
motor RightIntake1 = motor(RightIntake1Port, ratio6_1, false);

motor L1M = LeftChassis1; // Backwards Compatibility
motor L2M = LeftChassis2;
motor R1M = RightChassis2;
motor R2M = RightChassis2;
motor FRM = FrontRollers;
motor LRM = LaunchRoller;
motor LIM = LeftIntake1;
motor RIM = RightIntake1;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int calibrate(void) {
  Gyro.calibrate();
  { // reset Rotation
    L1M.resetRotation();
    L2M.resetRotation();
    R1M.resetRotation();
    R2M.resetRotation();
    FRM.resetRotation();
    LRM.resetRotation();
    LIM.resetRotation();
    RIM.resetRotation();
  }
  waitUntil(!Gyro.isCalibrating());
  Gyro.setHeading(0, degrees);
  return 0;
}

void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  task setup = task(calibrate);     // multitask
  waitUntil(!Gyro.isCalibrating()); // wait for the ending
  return;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

void diagnose(bool full) { // check tor motor disconnects
  if (full & !comp) {
    if (vexBatteryCapacityGet() < 60) {
      Brain.Screen.print("Battey ERROR\n");
    }
    Brain.Screen.print("Battery Percentage: ");
    Brain.Screen.print(vexBatteryCapacityGet());
    Brain.Screen.print("%");
    LeftChassis1.spin(fwd, 0.128 * 100, voltageUnits::volt);
    sleep(3000);
    LeftChassis1.stop();
    sleep(1000);
    LeftChassis2.spin(fwd, 0.128 * 100, voltageUnits::volt);
    sleep(3000);
    LeftChassis2.stop();
    sleep(1000);
    RightChassis2.spin(fwd, 0.128 * 100, voltageUnits::volt);
    sleep(3000);
    RightChassis2.stop();
    sleep(1000);
    RightChassis1.spin(fwd, 0.128 * 100, voltageUnits::volt);
    sleep(3000);
    RightChassis1.stop();
    while (!Controller1.ButtonA.pressing()) {
      sleep(pause);
    }
  }
}

void runAuto() { waitUntil(Gyro.heading() > 90); }

void autonomous(void) {
  diagnose(deb); // change debug in config
  runAuto();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
float Ch1, Ch2, Ch3, Ch4;
bool L1, L2, R1, R2, BtnA, BtnB, BtnX, BtnY, BtnU, BtnD, BtnL,
    BtnR; // Buttons for the controller

int getController(void) {
  do {
    Ch1 = Controller1.Axis1.value() * 1;
    Ch2 = Controller1.Axis2.value() * 1.2;
    Ch3 = Controller1.Axis3.value() * 1.2;
    Ch4 = Controller1.Axis4.value() * 1.2;
    L1 = Controller1.ButtonL1.pressing();
    L2 = Controller1.ButtonL2.pressing();
    R2 = Controller1.ButtonR1.pressing();
    R1 = Controller1.ButtonR2.pressing();
    BtnA = Controller1.ButtonA.pressing();
    BtnB = Controller1.ButtonB.pressing();
    BtnX = Controller1.ButtonX.pressing();
    BtnY = Controller1.ButtonY.pressing();
    BtnU = Controller1.ButtonUp.pressing();
    BtnD = Controller1.ButtonDown.pressing();
    BtnL = Controller1.ButtonLeft.pressing();
    BtnR = Controller1.ButtonRight.pressing();
    sleep(pause / 2); // faster refresh
  } while (true);
  return 0;
}

int move(void) {
  do {
    if (std::abs(Ch3) < 5)
      ChassisControl(Ch1 * 0.5, -Ch1 * 0.5);
    else if (std::abs(Ch3) > 5)
      ChassisControl((Ch3 + Ch1), (Ch3 - Ch1));
    else
      Stop(brake);
    sleep(pause);
  } while (true);
  return 0;
}

int ball() {
  do {
    if (R1)
      Intake(100);
    else if (R2)
      Intake(-100);
    else if (L1) {
      FrontRollersControl(100);
      LaunchRollerControl(100);
    } else if (L2) {
      FrontRollersControl(-100);
      LaunchRollerControl(-100);
    } else if (std::abs(Ch2) > std::abs(Ch1)) {
      LaunchRollerControl(Ch2);
      Intake(-1 * Ch2);
    } else if (BtnA) {
      FrontRollersControl(100);
      LaunchRollerControl(100);
    } else if (BtnB) {
      FrontRollersControl(-100);
      LaunchRollerControl(-100);
    } else {
      LeftIntake1.stop(hold);
      RightIntake1.stop(hold);
      FrontRollers.stop(hold);
      LaunchRoller.stop(hold);
    }
    sleep(pause);
  } while (true);
  return 0;
}

void usercontrol(void) {
  task button = task(getController); // update controller status
  task chassis = task(move);         // move chassis
  task suck = task(ball);            // move rollers
  do {
    sleep(pause * 2); // infinite loop
  } while (true);
  button.stop();
  chassis.stop();
  suck.stop();
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  do
    sleep(pause * 2);
  while (true); // Sleep the task for a short amount of time to prevent
                // wasted resources.
}
