#include "odometry.h"
#include "navigation.h"
#include "robot_config.h"
#include "utils.h"
#include "vex.h"

using namespace vex;

double globalX = 0.0;       
double globalY = 0.0;       
double globalRotation = 0.0; 
double RobotStartingHeading = 0.0; 

double prevLeftEncoder = 0.0;  
double prevRightEncoder = 0.0; 
double prevXEncoder = 0.0;     
double prevRotation = 0.0;     

bool xEncoderEnabled = true; 

enum RobotState { STATIONARY, TURNING, STRAIGHT };
RobotState currentState = STATIONARY; 

void setStartPosition(double startX, double startY, double startHeading) {
    globalX = startX;
    globalY = startY;
    RobotStartingHeading = startHeading - InertialSensor.rotation(degrees);
    double startX_deg = (startX / encoderWheelCircumferenceCM) * 360.0;
    double startY_deg = (startY / encoderWheelCircumferenceCM) * 360.0;
    passiveEncoderLeft.setPosition(startY_deg, vex::rotationUnits::deg);
    passiveEncoderRight.setPosition(startY_deg, vex::rotationUnits::deg);
    passiveEncoderX.setPosition(startX_deg, vex::rotationUnits::deg);
    
    prevLeftEncoder = startY_deg;
    prevRightEncoder = startY_deg;
    prevXEncoder = startX_deg;
}

void updateOdometry() {
    double leftEncoder = passiveEncoderLeft.position(vex::rotationUnits::deg);                            
    double rightEncoder = passiveEncoderRight.position(vex::rotationUnits::deg);                          
    double xEncoder = xEncoderEnabled ? passiveEncoderX.position(vex::rotationUnits::deg) : prevXEncoder; 
    double currentRotation = getAdjustedRotation();                                                        

    double deltaLeft = leftEncoder - prevLeftEncoder;                
    double deltaRight = rightEncoder - prevRightEncoder;             
    double deltaX = xEncoder - prevXEncoder;                         
    double deltaRotation = currentRotation - prevRotation;

    prevLeftEncoder = leftEncoder;   
    prevRightEncoder = rightEncoder; 
    prevXEncoder = xEncoder;         
    prevRotation = currentRotation;     

    double avgDeltaDistance = ((deltaLeft + deltaRight) / 2.0) * (encoderWheelCircumferenceCM / 360.0);
    double deltaXPos = 0.0, deltaYPos = 0.0;
    double headingRad = globalRotation * (M_PI / 180.0);    

    if (currentState == TURNING) {
        double deltaRotationRad = deltaRotation * (M_PI / 180.0); 
        if (fabs(deltaRotationRad) > 0.001) {
            double leftRadius = (deltaLeft * (encoderWheelCircumferenceCM / 360.0)) / deltaRotationRad;
            double rightRadius = (deltaRight * (encoderWheelCircumferenceCM / 360.0)) / deltaRotationRad;
            double avgRadius = (leftRadius + rightRadius) / 2.0;
            deltaYPos = (deltaX * (encoderWheelCircumferenceCM / 360.0));
            deltaXPos = avgRadius * (sin(headingRad + deltaRotationRad) - sin(headingRad));
        }
    } else if (currentState == STRAIGHT) {
        double lateralMovement = (deltaX * (encoderWheelCircumferenceCM / 360.0));
        deltaYPos = avgDeltaDistance * cos(headingRad) + lateralMovement * (-sin(headingRad));
        deltaXPos = avgDeltaDistance * sin(headingRad) + lateralMovement * cos(headingRad);
    }

    globalX += deltaXPos;                        
    globalY += deltaYPos;                        
    globalRotation = getAdjustedRotation();      
    double displayRotation = standardToModifiedCartesian(globalRotation);
    Brain.Screen.printAt(10, 20, "X: %.2f, Y: %.2f, Rotation: %.2f", globalX, globalY, displayRotation);
}

void calculatePathToTarget(double currentX, double currentY, double targetX, double targetY, double &distance, double &heading) {
    double deltaX = targetX - currentX;
    double deltaY = targetY - currentY;
    distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    heading = atan2(deltaY, deltaX) * 180.0 / M_PI;
}

void turnToPoint(double targetX, double targetY, double breakDistanceInDegrees, double minSpeed, double maxSpeed) {
    currentState = TURNING;
    updateOdometry();
    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double targetStandardHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    double currentStandardHeading = getAdjustedRotation();
    double headingError = targetStandardHeading - currentStandardHeading;
    headingError = fmod(headingError + 540.0, 360.0) - 180.0;
    double finalTargetHeading = currentStandardHeading + headingError;
    turnOdometry(finalTargetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);
    updateOdometry();
    currentState = STATIONARY;
}

void turnLeftToPoint(double targetX, double targetY, double breakDistanceInDegrees, double minSpeed, double maxSpeed) {
    currentState = TURNING;
    updateOdometry();
    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double targetAbsoluteHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    double currentHeading = getAdjustedRotation();
    double targetHeading = targetAbsoluteHeading;
    while (targetHeading <= currentHeading) targetHeading += 360.0;
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);
    updateOdometry();
    currentState = STATIONARY;
}

void turnRightToPoint(double targetX, double targetY, double breakDistanceInDegrees, double minSpeed, double maxSpeed) {
    currentState = TURNING;
    updateOdometry();
    double deltaX = targetX - globalX;
    double deltaY = targetY - globalY;
    double targetStandardHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
    double currentStandardHeading = getAdjustedRotation();
    double targetHeading = targetStandardHeading;
    while (targetHeading >= currentStandardHeading) targetHeading -= 360.0;
    turnOdometry(targetHeading, breakDistanceInDegrees, minSpeed, maxSpeed);
    updateOdometry();
    currentState = STATIONARY;
}

void forwardToPoint(double targetX, double targetY, double breakDistance, double minSpeed, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    updateOdometry();
    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);
    straightOdometry(distanceToTarget, breakDistance, targetHeading, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
    updateOdometry();
}

void backwardToPoint(double targetX, double targetY, double minSpeed, double breakDistance, double kp_heading, double ki_heading, double kd_heading, double accelHeadingScaling, double decelHeadingScaling, double approachHeadingScaling, double maxSpeed) {
    currentState = STRAIGHT;
    if (maxSpeed > 0) maxSpeed = -fabs(maxSpeed);
    updateOdometry();
    double distanceToTarget, targetHeading;
    calculatePathToTarget(globalX, globalY, targetX, targetY, distanceToTarget, targetHeading);
    targetHeading = targetHeading + 180.0;  
    distanceToTarget = -fabs(distanceToTarget);
    straightOdometry(distanceToTarget, breakDistance, targetHeading, minSpeed, kp_heading, ki_heading, kd_heading, accelHeadingScaling, decelHeadingScaling, approachHeadingScaling, maxSpeed);
    updateOdometry();
    currentState = STATIONARY;
}

OdometryTaskParams odometryParams = {false};
int odometryTask(void *params) {
    OdometryTaskParams *p = static_cast<OdometryTaskParams *>(params);
    while (p->isRunning) {
        updateOdometry();
        wait(10, msec);
    }
    return 0;
}

void startOdometryTask() {
    if (!odometryParams.isRunning) {
        odometryParams.isRunning = true;
        task odomTask(odometryTask, &odometryParams);
    }
}

void stopOdometryTask() { odometryParams.isRunning = false; }