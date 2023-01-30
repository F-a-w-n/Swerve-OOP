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
#include "Module.cpp"

// Start using namespaces so classes don't have to be referenced constantly.
using namespace frc;
using namespace std;
using namespace rev;

static class Swerve {
    public:

    //modules, takes CAN id's in order or drive, turn, and encoder
    Module FR{4, 3, 10};
    Module FL{2, 1 , 9};
    Module BR{7, 8 , 12};
    Module BL{5, 6 , 11};

    int coast = 15; // Controls speed of acceleration and decceleration.  Higher value leads to slower accel/deccel.

    // Remember old inputs for accel/deccel.
    double oldJoyMag = 0;
    double oldJoyAng = 0;
    double oldTurn = 0;

    // Used to fix veering with accel/deccel.
    double oldGyro = 0;
    double coastAngleMod=0;

    // Used to control the robots speed and how it moves.
    double driveTurnRatio = 0.7; // 0.7 means 70% of wheel power goes to strafe and 30% to rotating.
    double speedMod = 1;

    void periodic(float joyX, float joyY, double turn, double gyroYaw, bool robotPerspective, bool postSpeedNormalization) {
        // Zero joystick values if they are small enough.
        joyX = (abs(joyX)<0.05)? 0 : joyX;
        joyY = (abs(joyY)<0.05)? 0 : joyY;
        turn= (abs(turn)<0.05)? 0 : turn;

        double injoyMag = getJoyMag(joyX,joyY);
        double inj = getJoyAngle(joyX, joyY);

        // Modifies inputs for either pre or post speed normalization.
        if(!postSpeedNormalization){      //pre speed normalization - modifies the joystick values to clamp within -1 to 1 range, always the same ratio
            injoyMag*=driveTurnRatio;
            turn*=(1-driveTurnRatio);
        }else{                             //post speed normalization - similar idea but allows for varying ratio of drive to turn, can go full speed in either but keeps it clamped still
            if(injoyMag+turn>1){            //if it needs to be clamped
                if(injoyMag>driveTurnRatio){  //if the drive ratio is too high, brings it down
                    injoyMag=1-min(turn,1-driveTurnRatio);
                }
                if(turn>1-driveTurnRatio){   //if the turn ratio is too high, brings it down
                    turn=1-min(injoyMag,driveTurnRatio);
                }
            }
        }

        // Modifes joy inputs to allow for accel/deccel.
        auto [joyMag,joyAng] = addVectors(injoyMag/coast,injoyAng,oldJoyMag*(coast-1)/coast,oldJoyAng);   //certain ratio of the vectors is the current input, the rest is old vector, slowly ramps up and down
        turn=turn/coast+oldTurn*(coast-1)/coast;

        // Modifies the drive angle for field perspective or not for robot perspective.
        double angle;
        if (!robotPerspective) {
        angle = normalizeAngle2(360.0-(normalizeGyro(gyroYaw)-joyAng));  //angle relative to gyroscope reading
        } else {
        angle=joyAng;
        }

        // Vector addition for wheel angles and magnitudes. format: (F- front, B- back, W- wheel, M - magnitude, A- angle)
        auto [FLWM,FLWA] = addVectors(joyMag,angle,turn,45.0);
        auto [FRWM,FRWA] = addVectors(joyMag,angle,turn,135.0);
        auto [BLWM,BLWA] = addVectors(joyMag,angle,turn,315.0);
        auto [BRWM,BRWA] = addVectors(joyMag,angle,turn,225.0);

        // Sets the wheels to their magnitudes.
        FL.setWheelDrive(FLWA,FLWM);
        FR.setWheelDrive(FRWA,FRWM);
        BL.setWheelDrive(BLWA,BLWM);
        BR.setWheelDrive(BRWA,BRWM);

        // Sets the wheels to their angles.
        BR.setTurn(BRWA);
        BL.setTurn(BLWA);
        FR.setTurn(FRWA);
        FL.setTurn(FLWA);

        // Sets the old input values for accel/deccel.
        oldJoyMag = joyMag;
        oldJoyAng = joyAng;
        oldTurn = turn;
        
    }

    tuple<double,double> addVectors(double mag1, double ang1, double mag2, double ang2){
        
        double x1 = sin(rad(ang1))*mag1;
        double y1 = -cos(rad(ang1))*mag1;

        double x2 = sin(rad(ang2))*mag2;
        double y2 = -cos(rad(ang2))*mag2;

        double xFinal = x1+x2;
        double yFinal = y1+y2;

        double magFinal = pow(pow(xFinal,2)+pow(yFinal,2),0.5); //a^2 + b^2 = c^2, then square roots c

        double angFinal=0.0;
        //calculates slope based on the x and y coordinates, then uses inverse tan to get the angle
        if(xFinal>0.0 && yFinal<0.0){                   //270 - 360 degrees
        angFinal = deg(atan(xFinal/(-yFinal)));
        }
        else if(xFinal>0.0 && yFinal>0.0){              //0 - 90 degrees
        angFinal = deg(atan(yFinal/xFinal)+M_PI/2.0);
        }
        else if(xFinal<0.0 && yFinal>0.0){              //90 - 180 degrees
        angFinal = deg(atan((-xFinal)/yFinal)+M_PI);
        }
        else if(xFinal<0.0 && yFinal<0.0){              //180 - 270 degrees
        angFinal = deg(atan((-yFinal)/(-xFinal))+M_PI*1.5);
        }

        return make_tuple(magFinal,angFinal);
    }

    // Normalizes an angle to between 0 and 360.
    double normalizeAngle2(double angle){
        return fmod(fmod(angle,360.0)+360.0,360.0);
    }

    // Normalizes the gyro to between 0 and 360.
    double normalizeGyro(double angle){
        if(angle<0){
        return 360.0+angle;
        }
        return angle;
    }


    // Gets the angle a joystick is pointing at.
    float getJoyAngle(float joyX, float joyY){
        float angleVal=0;
        if (abs(joyX)<=0.05 && abs(joyY)<=0.05){return 0;}    //if the joystick is stationary
        if (joyX==0 && joyY<=0){return 0;}                    //when directly on the axes, prevents tan and inverse tan issues
        if (joyX==0 && joyY>0){return 180;}
        if (joyX<0 && joyY==0){return 270;}
        if (joyX>0 && joyY==0){return 90;}
        if (joyX>0){
        angleVal= (joyY<0)? (atan(joyX/abs(joyY))*180.0/M_PI) : (180-atan(joyX/joyY)*180.0/M_PI);
        return angleVal;
        }
        angleVal= (joyY<0)? (360.0-atan(abs(joyX)/abs(joyY))*180.0/M_PI) : (atan(abs(joyX)/joyY)*180.0/M_PI+180.0);
        return angleVal;    
    }

    // Gets the magnitude of a joystick.
    double getJoyMag(double joyX, double joyY){
        return min(1.0,sqrt(pow(joyX,2)+pow(joyY,2)));  //pythagorean theorem to find the actual distance, clamped to a max of 1
    }

    // Converts an angle from degrees to radians.
    double rad(double angle){
        return angle * (M_PI/180.0);
    }

    // Converts an angle from radians to degrees.
    double deg(double angle){
        return angle * (180.0/M_PI);
    }

};