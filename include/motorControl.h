#include "vex.h"
float POW;

void ChassisControl(float lPower, float rPower);
void Stop(vex::brakeType type = vex::brake);
void FrontRollersControl(int power);
void LaunchRollerControl(int power, vex::brakeType type = vex::brake);
void Intake(float power);
void Shuffle(float timeLimit, float distance);
// void motorturn(int a, int b);
// void spread();
// bool goForward(int power, float target, float timeLimit);
// bool goBackward(int power, float target, float timeLimit);
// bool turnLeft(int power, float target, float timeLimit);
// bool goForwardWithA(int power, float target, float timeLimit, float a);
// bool goBackwardWithA(int power, float target, float timeLimit, float a);
// bool turnLeftWithA(int power, float target, float timeLimit, float a);
// bool turnRightWithA(int power, float target, float timeLimit, float a);
// bool turnRight(int power, float target, float timeLimit);
// void backToWall(float power, int dis1, int dis2, int dis3, int dis4, int
// time,
//                bool left);
// bool turnLeftWithGyro(int power, float target, float timeLimit);
// bool turnRightWithGyro(int power, float target, float timeLimit);
// void turn(float power, float degrees, int timeLimit);
// float p(float power, float turn, float low);

//#include "C:/Program Files (x86)/VEX
// Robotics/VEXcode/sdk/vexv5/include/vex_global.h"

// VEXcode devices
extern motor LeftChassis2;
extern motor LeftChassis1;
extern motor RightChassis2;
extern motor RightChassis1;
extern motor FrontRollers;
extern motor LaunchRoller;
extern motor LeftIntake1;
extern motor RightIntake1;
extern vex::vision::signature Vision__ZHUIZONG;
extern vex::vision::signature Vision__SIG_2;
extern vex::vision::signature Vision__SIG_3;
extern vex::vision::signature Vision__SIG_4;
extern vex::vision::signature Vision__SIG_5;
extern vex::vision::signature Vision__SIG_6;
extern vex::vision::signature Vision__SIG_7;
extern vision Vision;
extern gyro Gyro;
extern controller Controller1;
extern bumper BumperH;

// A global instance of brain used for printing to the V5 Brain screen

void ChassisControl(float lPower, float rPower) {
  LeftChassis1.spin(fwd, 0.13 * lPower, voltageUnits::volt);
  LeftChassis2.spin(fwd, 0.13 * lPower, voltageUnits::volt);

  RightChassis1.spin(fwd, 0.13 * rPower, voltageUnits::volt);
  RightChassis2.spin(fwd, 0.13 * rPower, voltageUnits::volt);
}

void Stop(brakeType type) {
  LeftChassis1.stop(type);
  LeftChassis2.stop(type);
  RightChassis1.stop(type);
  RightChassis2.stop(type);
}
//================================================================================
void FrontRollersControl(int power) {
  if (power == 0) {
    FrontRollers.stop(hold);
  } else {
    FrontRollers.spin(fwd, 0.128 * power, voltageUnits::volt);
  }
}
//====================================================================================
void LaunchRollerControl(int power, brakeType type) {
  if (power == 0) {
    LaunchRoller.stop(hold);
  } else {
    LaunchRoller.spin(fwd, 0.128 * power, voltageUnits::volt);
  }
}
//====================================================================================
void Intake(float power) {
  power *= 1.5;
  if (power == 0) {
    LeftIntake1.stop(hold);
    RightIntake1.stop(hold);
  } else {
    LeftIntake1.spin(fwd, 0.128 * power, voltageUnits::volt);
    // LeftIntake2.spin(fwd, 0.12 * power, voltageUnits::volt);
    RightIntake1.spin(fwd, 0.128 * power, voltageUnits::volt);
    // RightIntake2.spin(fwd, 0.12 * power, voltageUnits::volt);
    FrontRollersControl(power * -1);
  }
}

// void spread() { Intake(100); }
//
// bool goForward(int power, float target, float timeLimit) {
//  float errL = 0.0;
//  float errR = 0.0;
//  float err_lastL = 0.0;
//  float err_lastR = 0.0;
//  float voltL = 0.0;
//  float voltR = 0.0;
//  float integralL = 0.0;
//  float integralR = 0.0;
//  float indexL = 0.0;
//  float indexR = 0.0;
//  LeftChassis2.resetRotation();
//  RightChassis2.resetRotation();
//  Brain.resetTimer();
//  while (Brain.timer(msec) < timeLimit) {
//    float curL = LeftChassis2.rotation(deg);
//    float curR = RightChassis2.rotation(deg);
//    errL = target - curL;
//    errR = target - curR;
//
//    if (abs((int)errL) > target) {
//      indexL = 0;
//    } else if (abs((int)errL) < target * KI_START_PERCENT) {
//      indexL = 1;
//      integralL += errL / 2;
//    } else {
//      indexL = (target - abs((int)errL)) / KI_INDEX_PAR;
//      integralL += errL / 2;
//    }
//
//    if (abs((int)errR) > target) {
//      indexR = 0;
//    } else if (abs((int)errR) < target * KI_START_PERCENT) {
//      indexR = 1;
//      integralR += errR / 2;
//    } else {
//      indexR = (target - abs((int)errR)) / KI_INDEX_PAR;
//      integralR += errR / 2;
//    }
//
//    voltL = KP * errL + indexL * KI * integralL + KD * (errL - err_lastL);
//    voltR = KP * errR + indexR * KI * integralR + KD * (errR - err_lastR);
//    if (voltL < 1 || voltR < 1)
//      return true;
//    integralL += errL / 2;
//    integralR += errR / 2;
//    err_lastL = errL;
//    err_lastR = errR;
//
//    float rpmAdjust =
//        1.5 * (LeftChassis1.velocity(rpm) - RightChassis2.velocity(rpm)) *
//        (curL / target);
//    float rotationAdjust =
//        1.5 * (LeftChassis2.rotation(deg) - RightChassis2.rotation(deg)) *
//        (curL / target);
//    ChassisControl(CONSTRAIN(voltL - rpmAdjust - rotationAdjust, 0, power),
//                   CONSTRAIN(voltR + rpmAdjust + rotationAdjust, 0, power));
//  }
//  return false;
//}
//
// bool goForwardWithA(int power, float target, float timeLimit, float a) {
//  float errL = 0.0;
//  float errR = 0.0;
//  float err_lastL = 0.0;
//  float err_lastR = 0.0;
//  float voltL = 0.0;
//  float voltR = 0.0;
//  float integralL = 0.0;
//  float integralR = 0.0;
//  float indexL = 0.0;
//  float indexR = 0.0;
//  LeftChassis2.resetRotation();
//  RightChassis2.resetRotation();
//  Brain.resetTimer();
//  while (Brain.timer(msec) < timeLimit) {
//    float curL = LeftChassis2.rotation(deg);
//    float curR = RightChassis2.rotation(deg);
//    errL = target - curL;
//    errR = target - curR;
//
//    if (abs((int)errL) > target) {
//      indexL = 0;
//    } else if (abs((int)errL) < target * KI_START_PERCENT) {
//      indexL = 1;
//      integralL += errL / 2;
//    } else {
//      indexL = (target - abs((int)errL)) / KI_INDEX_PAR;
//      integralL += errL / 2;
//    }
//
//    if (abs((int)errR) > target) {
//      indexR = 0;
//    } else if (abs((int)errR) < target * KI_START_PERCENT) {
//      indexR = 1;
//      integralR += errR / 2;
//    } else {
//      indexR = (target - abs((int)errR)) / KI_INDEX_PAR;
//      integralR += errR / 2;
//    }
//
//    voltL = KP * errL + indexL * KI * integralL + KD * (errL - err_lastL);
//    voltR = KP * errR + indexR * KI * integralR + KD * (errR - err_lastR);
//    if (voltL < 1 || voltR < 1)
//      return true;
//    integralL += errL / 2;
//    integralR += errR / 2;
//    err_lastL = errL;
//    err_lastR = errR;
//
//    POW = power / a * Brain.timer(msec) / 1000;
//    if (POW > 100)
//      POW = 100;
//    float rpmAdjust =
//        1.5 * (LeftChassis1.velocity(rpm) - RightChassis2.velocity(rpm)) *
//        (curL / target);
//    float rotationAdjust =
//        1.5 * (LeftChassis2.rotation(deg) - RightChassis2.rotation(deg)) *
//        (curL / target);
//    ChassisControl(CONSTRAIN(voltL - rpmAdjust - rotationAdjust, 0, POW),
//                   CONSTRAIN(voltR + rpmAdjust + rotationAdjust, 0, POW));
//  }
//  return false;
//}
//
// bool goBackward(int power, float target, float timeLimit) {
//  float errL = 0.0;
//  float errR = 0.0;
//  float err_lastL = 0.0;
//  float err_lastR = 0.0;
//  float voltL = 0.0;
//  float voltR = 0.0;
//  float integralL = 0.0;
//  float integralR = 0.0;
//  float indexL = 0.0;
//  float indexR = 0.0;
//  LeftChassis2.resetRotation();
//  RightChassis2.resetRotation();
//  Brain.resetTimer();
//  //==================================================================================
//  while (Brain.timer(msec) < timeLimit) {
//    float curL = LeftChassis2.rotation(deg);
//    float curR = RightChassis2.rotation(deg);
//    errL = target + curL;
//    errR = target + curR;
//
//    if (abs((int)errL) > target) {
//      indexL = 0;
//    } else if (abs((int)errL) < target * KI_START_PERCENT) {
//      indexL = 1;
//      integralL += errL / 2;
//    } else {
//      indexL = (target - abs((int)errL)) / KI_INDEX_PAR;
//      integralL += errL / 2;
//    }
//
//    if (abs((int)errR) > target) {
//      indexR = 0;
//    } else if (abs((int)errR) < target * KI_START_PERCENT) {
//      indexR = 1;
//      integralR += errR / 2;
//    } else {
//      indexR = (target - abs((int)errR)) / KI_INDEX_PAR;
//      integralR += errR / 2;
//    }
//
//    voltL = 0.5 * KP * errL + indexL * KI * integralL + KD * (errL -
//    err_lastL); voltR = 0.5 * KP * errR + indexR * KI * integralR + KD * (errR
//    - err_lastR); if (voltL < 1 || voltR < 1)
//      return true;
//    integralL += errL / 2;
//    integralR += errR / 2;
//    err_lastL = errL;
//    err_lastR = errR;
//
//    float rpmAdjust =
//        1.5 * (LeftChassis1.velocity(rpm) - RightChassis2.velocity(rpm)) *
//        (-curL / target);
//    float rotationAdjust =
//        0.5 * (LeftChassis2.rotation(deg) - RightChassis2.rotation(deg)) *
//        (-curL / target);
//    ChassisControl(CONSTRAIN(-voltL + rpmAdjust + rotationAdjust, -power, 0),
//                   CONSTRAIN(-voltR - rpmAdjust - rotationAdjust, -power, 0));
//  }
//  return false;
//}
// bool goBackwardWithA(int power, float target, float timeLimit, float a) {
//  float errL = 0.0;
//  float errR = 0.0;
//  float err_lastL = 0.0;
//  float err_lastR = 0.0;
//  float voltL = 0.0;
//  float voltR = 0.0;
//  float integralL = 0.0;
//  float integralR = 0.0;
//  float indexL = 0.0;
//  float indexR = 0.0;
//  LeftChassis2.resetRotation();
//  RightChassis2.resetRotation();
//  Brain.resetTimer();
//  //==================================================================================
//  while (Brain.timer(msec) < timeLimit) {
//    float curL = LeftChassis2.rotation(deg);
//    float curR = RightChassis2.rotation(deg);
//    errL = target + curL;
//    errR = target + curR;
//
//    if (abs((int)errL) > target) {
//      indexL = 0;
//    } else if (abs((int)errL) < target * KI_START_PERCENT) {
//      indexL = 1;
//      integralL += errL / 2;
//    } else {
//      indexL = (target - abs((int)errL)) / KI_INDEX_PAR;
//      integralL += errL / 2;
//    }
//
//    if (abs((int)errR) > target) {
//      indexR = 0;
//    } else if (abs((int)errR) < target * KI_START_PERCENT) {
//      indexR = 1;
//      integralR += errR / 2;
//    } else {
//      indexR = (target - abs((int)errR)) / KI_INDEX_PAR;
//      integralR += errR / 2;
//    }
//
//    voltL = 0.5 * KP * errL + indexL * KI * integralL + KD * (errL -
//    err_lastL); voltR = 0.5 * KP * errR + indexR * KI * integralR + KD * (errR
//    - err_lastR); if (voltL < 1 || voltR < 1)
//      return true;
//    integralL += errL / 2;
//    integralR += errR / 2;
//    err_lastL = errL;
//    err_lastR = errR;
//
//    POW = power / a * Brain.timer(msec) / 1000;
//    if (POW > 100) {
//      POW = 100;
//    }
//
//    float rpmAdjust =
//        1.5 * (LeftChassis1.velocity(rpm) - RightChassis2.velocity(rpm)) *
//        (-curL / target);
//    float rotationAdjust =
//        0.5 * (LeftChassis2.rotation(deg) - RightChassis2.rotation(deg)) *
//        (-curL / target);
//    ChassisControl(CONSTRAIN(-voltL + rpmAdjust + rotationAdjust, -POW, 0),
//                   CONSTRAIN(-voltR - rpmAdjust - rotationAdjust, -POW, 0));
//  }
//  return false;
//}
////===================================================================================================================
// bool turnLeft(int power, float target, float timeLimit) {
//  float errL = 0.0;
//  float errR = 0.0;
//  float err_lastL = 0.0;
//  float err_lastR = 0.0;
//  float voltL = 0.0;
//  float voltR = 0.0;
//  float integralL = 0.0;
//  float integralR = 0.0;
//  float indexL = 0.0;
//  float indexR = 0.0;
//  LeftChassis2.resetRotation();
//  RightChassis2.resetRotation();
//  while (1) {
//    float curL = LeftChassis2.rotation(deg);
//    float curR = RightChassis2.rotation(deg);
//    errL = target + curL;
//    errR = target - curR;
//
//    if (abs((int)errL) > target) {
//      indexL = 0;
//    } else if (abs((int)errL) < target * KI_START_PERCENT) {
//      indexL = 1;
//      integralL += errL / 2;
//    } else {
//      indexL = (target - abs((int)errL)) / KI_INDEX_PAR;
//      integralL += errL / 2;
//    }
//
//    if (abs((int)errR) > target) {
//      indexR = 0;
//    } else if (abs((int)errR) < target * KI_START_PERCENT) {
//      indexR = 1;
//      integralR += errR / 2;
//    } else {
//      indexR = (target - abs((int)errR)) / KI_INDEX_PAR;
//      integralR += errR / 2;
//    }
//
//    voltL = KP * errL + indexL * KI * integralL + KD * (errL - err_lastL);
//    voltR = KP * errR + indexR * KI * integralR + KD * (errR - err_lastR);
//    if (voltL < 0.01 || voltR < 0.01)
//      return true;
//    integralL += errL / 2;
//    integralR += errR / 2;
//    err_lastL = errL;
//    err_lastR = errR;
//
//    float rpmAdjust =
//        1.0 * (LeftChassis1.velocity(rpm) - RightChassis2.velocity(rpm));
//    float rotationAdjust =
//        0; // 2.0 * (LeftMotor2.rotation(deg) - RightMotor2.rotation(deg));
//    ChassisControl(CONSTRAIN(-voltL + rpmAdjust + rotationAdjust, -power, 0),
//                   CONSTRAIN(voltR + rpmAdjust + rotationAdjust, 0, power));
//  }
//  return false;
//}
//
////==================================================================================
// bool turnLeftWithA(int power, float target, float timeLimit, float a) {
//  float errL = 0.0;
//  float errR = 0.0;
//  float err_lastL = 0.0;
//  float err_lastR = 0.0;
//  float voltL = 0.0;
//  float voltR = 0.0;
//  float integralL = 0.0;
//  float integralR = 0.0;
//  float indexL = 0.0;
//  float indexR = 0.0;
//  LeftChassis2.resetRotation();
//  RightChassis2.resetRotation();
//  Brain.resetTimer();
//  //==================================================================================
//  while (Brain.timer(msec) < timeLimit) {
//    float curL = LeftChassis2.rotation(deg);
//    float curR = RightChassis2.rotation(deg);
//    errL = target + curL;
//    errR = target + curR;
//
//    if (abs((int)errL) > target) {
//      indexL = 0;
//    } else if (abs((int)errL) < target * KI_START_PERCENT) {
//      indexL = 1;
//      integralL += errL / 2;
//    } else {
//      indexL = (target - abs((int)errL)) / KI_INDEX_PAR;
//      integralL += errL / 2;
//    }
//
//    if (abs((int)errR) > target) {
//      indexR = 0;
//    } else if (abs((int)errR) < target * KI_START_PERCENT) {
//      indexR = 1;
//      integralR += errR / 2;
//    } else {
//      indexR = (target - abs((int)errR)) / KI_INDEX_PAR;
//      integralR += errR / 2;
//    }
//
//    voltL = 0.5 * KP * errL + indexL * KI * integralL + KD * (errL -
//    err_lastL); voltR = 0.5 * KP * errR + indexR * KI * integralR + KD * (errR
//    - err_lastR); if (voltL < 1 || voltR < 1)
//      return true;
//    integralL += errL / 2;
//    integralR += errR / 2;
//    err_lastL = errL;
//    err_lastR = errR;
//
//    POW = power / a * Brain.timer(msec) / 1000;
//    if (POW > 100)
//      POW = 100;
//    float rpmAdjust =
//        1.0 * (LeftChassis1.velocity(rpm) - RightChassis2.velocity(rpm));
//    float rotationAdjust =
//        0; // 2.0 * (LeftMotor2.rotation(deg) - RightMotor2.rotation(deg));
//    ChassisControl(CONSTRAIN(-voltL + rpmAdjust + rotationAdjust, -POW, 0),
//                   CONSTRAIN(voltR + rpmAdjust + rotationAdjust, 0, POW));
//  }
//  return false;
//}
// bool turnRight(int power, float target, float timeLimit) {
//  float errL = 0.0;
//  float errR = 0.0;
//  float err_lastL = 0.0;
//  float err_lastR = 0.0;
//  float voltL = 0.0;
//  float voltR = 0.0;
//  float integralL = 0.0;
//  float integralR = 0.0;
//  float indexL = 0.0;
//  float indexR = 0.0;
//  LeftChassis2.resetRotation();
//  RightChassis2.resetRotation();
//  while (1) {
//    float curL = LeftChassis2.rotation(deg);
//    float curR = RightChassis2.rotation(deg);
//    errL = target - curL;
//    errR = target + curR;
//    if (abs((int)errL) > target) {
//      indexL = 0;
//    } else if (abs((int)errL) < target * KI_START_PERCENT) {
//      indexL = 1;
//      integralL += errL / 2;
//    } else {
//      indexL = (target - abs((int)errL)) / KI_INDEX_PAR;
//      integralL += errL / 2;
//    }
//
//    if (abs((int)errR) > target) {
//      indexR = 0;
//    } else if (abs((int)errR) < target * KI_START_PERCENT) {
//      indexR = 1;
//      integralR += errR / 2;
//    } else {
//      indexR = (target - abs((int)errR)) / KI_INDEX_PAR;
//      integralR += errR / 2;
//    }
//
//    voltL = KP * errL + indexL * KI * integralL + KD * (errL - err_lastL);
//    voltR = KP * errR + indexR * KI * integralR + KD * (errR - err_lastR);
//    if (voltL < 0.01 || voltR < 0.01)
//      return true;
//    integralL += errL / 2;
//    integralR += errR / 2;
//    err_lastL = errL;
//    err_lastR = errR;
//    float rpmAdjust =
//        1.0 * (LeftChassis1.velocity(rpm) - RightChassis2.velocity(rpm));
//    float rotationAdjust =
//        0; // 2.0 * (LeftMotor2.rotation(deg) - RightMotor2.rotation(deg));
//    ChassisControl(CONSTRAIN(voltL - rpmAdjust - rotationAdjust, 0, power),
//                   CONSTRAIN(-voltR - rpmAdjust - rotationAdjust, -power, 0));
//
//    sleep(5);
//  }
//  return false;
//}
////=================================================================================================
//
// float p(float power, float turn, float low) {
//  int po = 0;
//  if (turn > power)
//    po = power;
//  else if (turn < low)
//    po = low;
//  else
//    po = turn;
//  return po;
//}
////=========================================================================
// void turn(float power, float degrees, int timeLimit) {
//  float Kp = 0.5;
//  float Ki = 0.3;
//  float Kd = 0.3;
//  float error;
//  float lastError;
//  float P, I, D;
//  float turn;
//  float startTime = Brain.timer(timeUnits::msec);
//
//  while (Brain.timer(vex::timeUnits::msec) - startTime < timeLimit) {
//    error = degrees - Gyro.value(vex::rotationUnits::deg);
//    P = error * Kp;
//    I = (I + error) * Ki;
//    D = (error - lastError) * Kd;
//    lastError = error;
//    turn = P + I + D;
//    ChassisControl(turn, -turn);
//  }
//}
//