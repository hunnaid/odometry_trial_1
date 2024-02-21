#include "vex.h"
#include "odom.h"
#include "robot-config.h"

using namespace vex;

// GLOBAL COORDINATES
double Odom::globalX = 0.0;
double Odom::globalY = 0.0;
double Odom::globalAngle = 0.0;
double Odom::prevGlobalX = 0.0;
double Odom::prevGlobalY = 0.0;

// LOCAL COORDINATES
Point Odom::localDeltaPoint = {0, 0};

// SENSOR VALUES
// motor values
double Odom::leftfWheelPos = 0.0;
double Odom::rightfWheelPos = 0.0;
double Odom::leftbWheelPos = 0.0;  // Separate left back wheel motor
double Odom::rightbWheelPos = 0.0; // Separate right back wheel motor
// angle
double Odom::currentAngle = imu.heading(degrees);
double Odom::prevAngle = 0.0;

// ODOMETRY FUNCTIONS
void Odom::updateSensors() {
  // Replace encoder values with motor values
  double leftMotorDelta = leftfWheelPos - prevLeftfWheelPos;
  double rightMotorDelta = rightfWheelPos - prevRightfWheelPos;
  double leftBackMotorDelta = leftbWheelPos - prevLeftbWheelPos;  // Separate right back wheel motor
  double rightBackMotorDelta = rightbWheelPos - prevRightbWheelPos; // Separate left back wheel motor

  // Update angle
  deltaAngle = currentAngle - prevAngle;
  prevAngle = currentAngle;

  // Update motor values
  double leftWheelDelta = leftMotorDelta;
  double rightWheelDelta = rightMotorDelta;
  double backWheelDelta = (leftBackMotorDelta + rightBackMotorDelta) / 2; // Average of left and right back wheels

  // Polar coordinates
  localDeltaPoint.x = (deltaAngle + (backWheelDelta / backEncOffset)) * backEncOffset;
  localDeltaPoint.y = (leftWheelDelta + rightWheelDelta) / 2;

  // Cartesian coordinates
  globalX += (localDeltaPoint.y * sin(prevAngle + deltaAngle / 2)) + (localDeltaPoint.x * cos(prevAngle + deltaAngle / 2));
  globalY += (localDeltaPoint.y * cos(prevAngle + deltaAngle / 2)) - (localDeltaPoint.x * sin(prevAngle + deltaAngle / 2));
  globalAngle = currentAngle;

  // Update previous motor values
  prevLeftfWheelPos = leftfWheelPos;
  prevRightfWheelPos = rightfWheelPos;
  prevLeftbWheelPos = leftbWheelPos;
  prevRightbWheelPos = rightbWheelPos;
}

void Odom::reset() {
  leftfWheelPos = 0.0;
  rightfWheelPos = 0.0;
  leftbWheelPos = 0.0;
  rightbWheelPos = 0.0;
  prevAngle = 0.0;
  prevGlobalX = 0.0;
  prevGlobalY = 0.0;
}

void Odom::setPosition(double newX, double newY, double newAngle) {
  reset();
  prevAngle = newAngle;
  prevGlobalX = newX;
  prevGlobalY = newY;
}

// ODOMETRY THREAD
int Odom::Odometry() {
  while (true) {
    Odom::updateSensors();
    // Print or use globalX, globalY, globalAngle as needed

    this_thread::sleep_for(10);
  }
  return 0;
}
