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

// Start using namespaces so classes don't have to be referenced constantly.
using namespace frc;
using namespace std;
using namespace rev;

class Module {
    public:

    CANSparkMax Drive;
    CANSparkMax Turn;
    CANCoder Encoder;
    frc2::PIDController PID{0.0125, 0, 0.00025};

    Module(int DriveCAN, int TurnCAN, int Enc) {
        Drive = CANSparkMax(DriveCAN, CANSparkMax::MotorType::kBrushless);
        Turn = CANSparkMax(TurnCAN, CANSparkMax::MotorType::kBrushless);
        Encoder = CANCoder(Enc);
        PID.EnableContinuousInput(0,180);
        Drive.Set(0);
        Turn.Set(0);
    }

    void setWheelDrive(double ang, double power){
        double wheelPos = normalizeAngle2(Encoder.GetAbsolutePosition());
        // Find the direction the wheel should be spinning since sometimes it has to go backwards.
        int dir;
        if (abs(wheelPos-ang)<90.0){
        dir=1;
        }
        else if (360.0-wheelPos+ang<90.0){
        dir=1;
        }
        else if (360.0-ang+wheelPos<90.0){
        dir=1;
        }
        else{                             //rather than turning 180 degrees, just runs it backwards
        dir=-1;
        }
        // Sets the power of the motor in a given direction with the speed modifier.
        Drive.Set(power*dir*speedMod);
    }

    void setTurn(float angle) {
        float turnPow = PID.Calculate(normalizeAngle(Encoder.GetAbsolutePosition()), angle);
        float turnPow = clamp(turnPow,0.4),-0.4));
        Turn.Set(turnPow);

    }

    // Normalizes an angle to between 0 and 180.
    double normalizeAngle(double angle){
        while (angle>=360.0){   //ensures angle is clamped to 360 degrees
        angle-=360.0;
        }
        if (angle>=180.0){      //afterwards ensures the angle is within 0-180 degrees (only has to check once since it's withing 0-360 already)
        angle-=180.0;
        }
        return angle;
    }
    double normalizeAngle2(double angle){
        return fmod(fmod(angle,360.0)+360.0,360.0);
    }

}