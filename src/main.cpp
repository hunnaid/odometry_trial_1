#include "vex.h"
#include "robot-config.h"
#include "odom.h"

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       hunnaid                                                   */
/*    Created:      1/29/2024, 8:33:41 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
controller Controller1 = controller(primary);

bool RemoteControlCodeEnabled = true;

int driveout=0;
int turnout=0;
int strafe=0;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

Odom odom{

};

void pre_auton(void) {
  imu.calibrate();
  waitUntil(6000);

  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  waitUntil(1000);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              PID Task                                     */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// constants
double kP = 0.0; // tune it
double kI = 0.0; // tune it
double kD = 0.0; // tune it
double turnkP = 0.0; // tune it
double turnkI = 0.0; // tune it
double turnkD = 0.0; // tune it

// auton
int shit = 200;
int balls = 0;

// errors
int err; // delta x
int prevErr = 0; // delta x earlier
int deriv; // delta err (derivative ---> err - prevErr) ---> speed
int totalErr = 0;

int turnErr; // delta x
int turnPrevErr = 0; // delta x earlier
int turnDeriv; // delta err (derivative ---> err - prevErr) ---> speed
int turnTotalErr = 0;

//bool resetSensors = false;


int pid() {
  bool ena = true;
  imu.calibrate();
  while (ena) {
    
    /*if (resetSensors) {
      resetSensors = false;
      RFMotor.setPosition(0, degrees);
      LFMotor.setPosition(0, degrees);
      RBMotor.setPosition(0, degrees);
      LBMotor.setPosition(0, degrees);
    }*/

    // all 4 motor pos
    int LFPos = LFMotor.position(degrees);
    int LBPos = LBMotor.position(degrees);
    int RFPos = RFMotor.position(degrees);
    int RBPos = RBMotor.position(degrees);

    //----------------- lateral movement ---------------------

    // all 4 motor avg pos
    int avgPos = (LFPos + LBPos + RFPos + RBPos) / 4;

    err = avgPos - shit;
    deriv = err - prevErr;
    totalErr += err;

    double mp = (err * kP + totalErr * kI + deriv * kD);

    // -----------------------------------------------------

    //----------------- turning movement ---------------------
    
    int turnDif = imu.value(); // gets value from inertial sensor

    turnErr = turnDif - balls;
    turnDeriv = turnErr - turnPrevErr;
    turnTotalErr += turnErr;

    double turnmp = (turnErr * turnkP + turnTotalErr * turnkI + turnDeriv * turnkD);

    // -----------------------------------------------------

    LFMotor.spin(forward, (mp+turnmp)*.12, volt); //add
    LBMotor.spin(forward, (mp+turnmp)*.12, volt); //subtract
    RFMotor.spin(forward, (mp-turnmp)*.12, volt); //subtract
    RBMotor.spin(forward, (mp-turnmp)*.12, volt); //add

    turnPrevErr = turnErr;
    vex::task::sleep(20);
  }

  return 0;
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

void autonomous(void) {

  vex::task FUCK(pid);
  //resetSensors = true;
  //shit = 300;
  //balls = 600;

  vex::task::sleep(1000);


  Brain.Screen.clearScreen();

  driveout=6;
  turnout=0;
  strafe=0;
  LFMotor.spin(forward, (driveout+turnout+strafe), volt);
  LBMotor.spin(forward, (driveout+turnout-strafe), volt);
  RFMotor.spin(forward, (driveout-turnout-strafe), volt);
  RBMotor.spin(forward, (driveout-turnout+strafe), volt);
  waitUntil(500);

  LFMotor.stop(hold);
  LBMotor.stop(hold);
  RFMotor.stop(hold); 
  RBMotor.stop(hold);

  waitUntil(250);


  driveout=0;
  turnout=0;
  strafe=6;
  LFMotor.spin(forward, (driveout+turnout+strafe), volt);
  LBMotor.spin(forward, (driveout+turnout-strafe), volt);
  RFMotor.spin(forward, (driveout-turnout-strafe), volt);
  RBMotor.spin(forward, (driveout-turnout+strafe), volt);
  waitUntil(500);

  LFMotor.stop(hold);
  LBMotor.stop(hold);
  RFMotor.stop(hold); 
  RBMotor.stop(hold);

  waitUntil(250);

  driveout=6;
  turnout=0;
  strafe=0;
  LFMotor.spin(reverse, (driveout+turnout+strafe), volt);
  LBMotor.spin(reverse, (driveout+turnout-strafe), volt);
  RFMotor.spin(reverse, (driveout-turnout-strafe), volt);
  RBMotor.spin(reverse, (driveout-turnout+strafe), volt);
  waitUntil(500);

  LFMotor.stop(hold);
  LBMotor.stop(hold);
  RFMotor.stop(hold); 
  RBMotor.stop(hold);

  waitUntil(250);

  driveout=0;
  turnout=0;
  strafe=6;
  LFMotor.spin(reverse, (driveout+turnout+strafe), volt);
  LBMotor.spin(reverse, (driveout+turnout-strafe), volt);
  RFMotor.spin(reverse, (driveout-turnout-strafe), volt);
  RBMotor.spin(reverse, (driveout-turnout+strafe), volt);
  waitUntil(500);

  LFMotor.stop(hold);
  LBMotor.stop(hold);
  RFMotor.stop(hold); 
  RBMotor.stop(hold);

  waitUntil(250);

  Brain.Screen.print("autonomous code");
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

void usercontrol(void) {

  Brain.Screen.clearScreen();
  imu.calibrate();

  // place driver control in this while loop
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.clearLine();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(Odom::globalX);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print(Odom::globalY);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print(imu.heading(degrees));

    //Drive
    driveout=Controller1.Axis3.position();
    turnout=Controller1.Axis1.position()*.7;  //modifier
    strafe=Controller1.Axis4.position();

    LFMotor.spin(forward, (driveout+turnout+strafe)*.12, volt); //add
    LBMotor.spin(forward, (driveout+turnout-strafe)*.12, volt); //subtract
    RFMotor.spin(forward, (driveout-turnout-strafe)*.12, volt); //subtract
    RBMotor.spin(forward, (driveout-turnout+strafe)*.12, volt); //add

    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  odom.Odometry();

  // Run the pre-autonomous function.
  pre_auton();
  //odom();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
  return 0;
}
