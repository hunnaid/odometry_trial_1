#pragma once

#include "vex.h"

using namespace vex;

struct Point {
  double x, y, angle;
};

class Odom {
public:
  // CONSTANTS
  static constexpr double track = 13.5;           // inches
  static constexpr double backWheelOffset = 5.75;  // inches
  static constexpr double backEncOffset = 2.0;     // placeholder value, replace with the actual offset
  static constexpr double wheelCircumference = 2.75; // inches

  // GLOBAL COORDINATES
  static double globalX;
  static double globalY;
  static double globalAngle;
  static double prevGlobalX;
  static double prevGlobalY;

  // LOCAL COORDINATES
  static Point localDeltaPoint;

  // SENSOR VALUES
  // motors
  static double leftfWheelPos;
  static double rightfWheelPos;
  static double leftbWheelPos;  // Separate left back wheel motor
  static double rightbWheelPos; // Separate right back wheel motor
  static double prevLeftfWheelPos;
  static double prevRightfWheelPos;
  static double prevLeftbWheelPos;
  static double prevRightbWheelPos;

  // angle
  static double currentAngle;
  static double prevAngle;
  static double deltaAngle;

  // ODOMETRY FUNCTIONS
  static void updateSensors();
  static void updatePosition();
  static void reset();
  static void setPosition(double newX, double newY, double newAngle);

  // ODOMETRY THREAD
  static int Odometry();
};
