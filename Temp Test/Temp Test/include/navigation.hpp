#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "robot_config.hpp"

void move(double distanceCM, double maxSpeed, int dir = 1); // 1 = forward, -1 = reverse
void smartMove(double distanceCM, double maxSpeed, int dir = 1, double wallStalledTimeMs = -1);

void pidStraight(double targetHeading, double targetDistanceCM, double speed,
                 double kp_heading = 0.6, double ki_heading = 0, double kd_heading = 0,
                 double distanceOffset = 5.0, pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_BRAKE);

void turn(double targetHeading,
          double breakDistanceInDegrees,
          double minSpeed = 17,
          double maxSpeed = 100);

void straight(double targetDistance,
              double breakDistance,
              double minSpeed = 17,
              double targetHeading = 0,
              double kp_heading = 0.2,
              double ki_heading = 0.0,
              double kd_heading = 0.0,
              double accelHeadingScaling = 0.25,
              double decelHeadingScaling = 0.25,
              double approachHeadingScaling = 0.25,
              double maxSpeed = 50);

void straightOdometry(double targetDistance,
                      double breakDistance,
                      double targetHeading = 0,
                      double minSpeed = 16,
                      double kp_heading = 0.4,
                      double ki_heading = 0.01,
                      double kd_heading = 0.05,
                      double accelHeadingScaling = 0.2,
                      double decelHeadingScaling = 0.2,
                      double approachHeadingScaling = 0.2,
                      double maxSpeed = 100);

void straightOdometryV2(double targetDistance,
                        double breakDistance,
                        double targetHeading = 0,
                        double minSpeed = 16,
                        double distanceTolerance = 6.0,
                        double kp_heading = 0.4,
                        double ki_heading = 0.01,
                        double kd_heading = 0.05,
                        double accelHeadingScaling = 0.2,
                        double decelHeadingScaling = 0.2,
                        double approachHeadingScaling = 0.2,
                        double maxSpeed = 100);

void smartStraight(double targetDistance,
                   double breakDistance,
                   double targetHeading = 0,
                   double minSpeed = 16,
                   double wallStalledTimeMs = 100,
                   double kp_heading = 0.4,
                   double ki_heading = 0.01,
                   double kd_heading = 0.05,
                   double accelHeadingScaling = 0.2,
                   double decelHeadingScaling = 0.2,
                   double approachHeadingScaling = 0.2,
                   double maxSpeed = 100);

void backward(double targetDistance,
              double breakDistance,
              double minSpeed = 17,
              double targetHeading = 0,
              double kp_heading = 1.7,
              double ki_heading = 0.0007,
              double kd_heading = 0.0025,
              double accelHeadingScaling = 0.275,
              double decelHeadingScaling = 0.2,
              double approachHeadingScaling = 0.2,
              double maxSpeed = 100);

void arcTurn(double targetDistance,
             double breakDistance,
             double minSpeed,
             double maxSpeed,
             double turnRadius,
             bool turnLeft);

void turnOdometry(double targetHeading,
                  double breakDistanceInDegrees,
                  double minSpeed = 25,
                  double maxSpeed = 100,
                  double exitTolerance = 16.0);

double launchControl(double targetDriverSpeed, pros::Motor& motor, pros::Rotation& encoder);

// Define the LaunchControl class
class LaunchControl {
public:
    LaunchControl(pros::Motor& motor, pros::Rotation& encoder, double slipThresholdValue = 1.1);
    double adjustSpeed(double targetPower);

private:
    pros::Motor& motor;
    pros::Rotation& encoder;
    const double slipThreshold;
    double motorRPM;
    double encoderRPMScaled;
};

// Define the tractionControl class
class tractionControl {
public:
    tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold);
    double tractionControlSpeed(double tractionMotorVoltage, double motorSpeed, double robotSpeed, double accelFactor);

private:
    double minSpeedVoltage;
    double maxSpeedVoltage;
    double slipThreshold;
};

// Adaptive ABS for deceleration phase
class adaptiveABS {
private:
    double lockThreshold;
    double decelStepVoltage;
    double lastAttemptedVoltage;
    bool wasLockedLastCycle;
    pros::motor_brake_mode_e_t currentBrakeMode;

public:
    adaptiveABS(double decelStepPercent, double lockThreshold);
    void initialize(double startingVoltage);
    double decelControlSpeed(double wheelSpeed, double robotSpeed);
    pros::motor_brake_mode_e_t getBrakeMode() { return currentBrakeMode; }
};

void pivotTurnOdometry(double targetHeading,
                       double breakDistanceInDegrees,
                       double minSpeed, double maxSpeed);

void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);
void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed);

void driveForward(double targetDistance,
                  double breakDistance = 10,
                  double targetHeading = 0,
                  double minSpeed = 24,
                  double kp_heading = 1.1,
                  double ki_heading = 0.005,
                  double kd_heading = 0,
                  double accelHeadingScaling = 0.1,
                  double decelHeadingScaling = 1,
                  double approachHeadingScaling = 0.3,
                  double maxSpeed = 100);

void driveBackward(double targetDistance,
                   double breakDistance = 10,
                   double targetHeading = 0,
                   double minSpeed = 24,
                   double kp_heading = 1.1,
                   double ki_heading = 0.005,
                   double kd_heading = 0,
                   double accelHeadingScaling = 0.1,
                   double decelHeadingScaling = 1,
                   double approachHeadingScaling = 0.3,
                   double maxSpeed = 100);

void turnRight(double absoluteTargetHeading,
               double breakDistance,
               double minSpeed = 25,
               double maxSpeed = 100,
               double exitTolerance = 16);

void turnLeft(double absoluteTargtHeading,
              double breakDistance,
              double minSpeed = 25,
              double maxSpeed = 100,
              double exitTolerance = 16);

void pidlessForward(double timeMs, double speedPct);

void driveForwardV2(double targetDistance,
                    double breakDistance,
                    double targetHeading,
                    double minSpeed,
                    double distanceTolerance,
                    double kp_heading,
                    double ki_heading,
                    double kd_heading,
                    double accelHeadingScaling,
                    double decelHeadingScaling,
                    double approachHeadingScaling,
                    double maxSpeed);

void driveBackwardV2(double targetDistance,
                     double breakDistance,
                     double targetHeading,
                     double minSpeed,
                     double distanceTolerance,
                     double kp_heading,
                     double ki_heading,
                     double kd_heading,
                     double accelHeadingScaling,
                     double decelHeadingScaling,
                     double approachHeadingScaling,
                     double maxSpeed);

#endif // NAVIGATION_HPP
