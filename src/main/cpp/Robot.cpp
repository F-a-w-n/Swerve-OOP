// Code for Swerve Drive base, written by Tyler Womack and Aditya Patel.
#include "frc/TimedRobot.h"
#include "rev/CANSparkMax.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <math.h>
#include <bits/stdc++.h>
#include <AHRS.h>
#include <frc/controller/PIDController.h>
#include <cmath>

#include "Swerve.cpp"

// Start using namespaces so classes don't have to be referenced constantly.
using namespace frc;
using namespace std;
using namespace rev;

class Robot: public TimedRobot {
  public:

    Joystick joy1{0}; // Initialize joystick.

    AHRS ahrs{SPI::kMXP}; // Gyro for getting the robot's angle
    
    Robot():
    {}

  
  void RobotInit() {}
  void RobotPeriodic() {}

  void AutonomousInit() {}

  void AutonomousPeriodic() {}

  void TeleopInit() {
      ahrs.Reset();

  }

  void TeleopPeriodic() {
      // Allows for a gyro reset mid match.
      if(joy1.GetRawButton(4)){
        ahrs.Reset();
      }  

      periodic(joy1.getRawAxis(0), joy1.getRawAxis(1), joy1.GetRawAxis(4), ahrs.GetYaw(), joy1.GetRawButton(6), joy1.GetRawButton(5))
  }

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return StartRobot<Robot>();
}
#endif