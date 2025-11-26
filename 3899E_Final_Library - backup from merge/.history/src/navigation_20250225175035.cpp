#include "navigation.h"
#include "robot_config.h"
#include "vex.h"
#include "utils.h"
#include "pid.h" // Ensure this line is included
#include <cmath>
#include <algorithm>
#include <iterator>
#include <cstring> // Include the cstring library for strcmp
#include "odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace vex;

enum MotionPhase
{
    READY,
    LAUNCH,
    CRUISE,
    DECELERATE,
    APPROACH,
    STOP

};

// Function to move the six wheel motors based on a given distance (in cm), max speed, and direction (default is forward)
void move(double distanceCM, double maxSpeed, vex::directionType dir)
{
    // Use the globally declared wheel circumference to calculate the number of rotations needed
    double targetRotations = distanceCM / wheelCircumferenceCM;

    // Set brake mode to brake for all motors
    // Stop all left and right motors using the array and a loop
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(brakeType::coast);  // Stop left motors
        rightMotor[i].setBrake(brakeType::coast); // Stop right motors
    }

    // Set motor velocities and move them for the calculated number of rotations
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);
        rightMotor[i].spinFor(dir, targetRotations, rotationUnits::rev, maxSpeed, velocityUnits::pct, false);
    }
}

void pidStraight(double targetHeading, double targetDistanceCM, double speed, double kp_heading, double ki_heading, double kd_heading, double distanceOffset, brakeType brakeMode)
{
    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    // Convert % Speed input to voltage with max voltage of 12
    double speedVoltage = speed * 0.01 * 12;

    // Initialize PID controller for heading
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();

    // Normalize the target heading
    double normTargetHeading = normalizeHeading(targetHeading);

    // Start debug info
    Brain.Screen.clearScreen();
    Brain.Screen.print("Starting pidStraight function...");
    double currentDistance = 0;

    // Loop to continuously adjust motor power based on PID control
    while (currentDistance < (targetDistanceCM - distanceOffset))
    {
        // while (currentDistance < (targetDistanceCM)) {
        // Get the current heading and normalize it
        double currentHeading = InertialSensor.heading();
        double normCurrentHeading = normalizeHeading(currentHeading);

        // Debug print the current heading
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Current Heading: %f", currentHeading);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Normalized Heading: %f", normCurrentHeading);

        // Debug print the target heading
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Target Heading: %f", targetHeading);
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print("Normalized Target: %f", normTargetHeading);

        // Calculate the heading correction using the PID controller
        double headingCorrection = headingPID.calculate(normTargetHeading, normCurrentHeading);

        // Adjust motor speeds based on the heading correction
        double leftMotorSpeed = speedVoltage + headingCorrection;  // Base speed + heading correction
        double rightMotorSpeed = speedVoltage - headingCorrection; // Base speed + heading correction

        // Debug print motor speeds
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Left Motor Speed: %f", leftMotorSpeed);
        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print("Right Motor Speed: %f", rightMotorSpeed);

        // Set motor speeds
        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, leftMotorSpeed, voltageUnits::volt);
            rightMotor[i].spin(forward, leftMotorSpeed, voltageUnits::volt);
        }

        // Print encoder value and distance covered
        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2 * (encoderWheelCircumferenceCM / 360.0);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Target Distance: %f cm", targetDistanceCM);
        Brain.Screen.setCursor(9, 1);
        Brain.Screen.print("Distance: %f cm", currentDistance);

        // Small delay
        task::sleep(20);
    }

    // Stop the motors with the specified brake mode
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }

    // Final debug message
    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("pidStraight finished.");
}

void turn(double turnDegrees, double breakDistanceInDegrees, double minSpeed, double maxSpeed)
{
    // Reset completion flags
    bool crossed180 = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    // Get heading offset at start
    headingOffset = 0;

    // Get starting position with offset conversion
    // double startingNormHeading = normHeading(convertHeading(InertialSensor.heading(degrees), headingOffset));  // Changing it so that we always start at 0 which it should and ignore any subsequent drifts.
    double startingNormHeading = 0;

    double normCurrentHeading = startingNormHeading;
    double currentDistanceInDegrees = 0;
    double targetDistanceInDegrees = normHeading(turnDegrees);

    Brain.Screen.printAt(10, 40, "Start Head: %.2f", startingNormHeading);
    Brain.Screen.printAt(10, 60, "Turn Target: %.2f", turnDegrees);

    // Convert % Speed input to voltage with max voltage of 12
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, targetDistanceInDegrees);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, targetDistanceInDegrees);
    double launchVoltage = std::copysign(5, targetDistanceInDegrees); // 3.5 is good
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), targetDistanceInDegrees);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxDriveMotorRPM = (maxSpeed * .01) * absoluteMaxRPM;

    double percentSpeedLoss = 0.2;

    // Maximum slip threshold for spot turns before reducing power
    // Range: 0-1, where:
    // 0 = No slip allowed (most conservative)
    // 1 = Full slip allowed (most aggressive)
    // 0.25 = 25% slip tolerance for balanced control
    // may need to go above 25% given built in difference between encoder and wheel spin speed
    const double SLIP_THRESHOLD_TRACTION = 0.35;
    const double SLIP_THRESHOLD_ABS = 0.5;

    double accelFactorLaunch = 1.4;
    double accelFactorCruise = 1.1;
    double encoderMotorScaleFactor = 0.857143; // 0.857143
    // double ABSLockThresholdSpotTurn = 0;
    // double totalMotorRadiansPerSecond = 0.0;
    // double minRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (minSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert minspeed to percentage first
    // double maxRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (maxSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert maxspeed to percentage first
    // double turnDirection = 1;
    double currentDegrees = 0;
    double currentNormHeading = 0;
    // double targetDriveVoltageLeft = 0;
    // double targetDriveVoltageRight = 0;
    // double motorRadiansPerSecondLeft[3];
    // double motorRadiansPerSecondRight[3];
    double leftMotorCMPerSecond[3];
    double rightMotorCMPerSecond[3];
    double leftMotorAngularRadians[3];
    double rightMotorAngularRadians[3];
    double leftMotorRPM[3] = {0, 0, 0};
    double rightMotorRPM[3] = {0, 0, 0};
    double robotRadiansPerSecond = 0;
    double averageMotorVoltage = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage}; // Initialize all elements to minimum launch speed
    vex::brakeType leftBrakeMode[3];
    vex::brakeType rightBrakeMode[3];
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    // double motorRadiansPerSecond[3] = {0, 0, 0};
    // double lowestMotorVoltage = 12;

    // MotionPhase currentPhase = READY;

    // Declaration for slip threshold
    ABSController ABSControllerLeft[3] = {
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS)};
    ABSController ABSControllerRight[3] = {
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS)};

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    tractionControl tractionControlRight[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    // launchControl (60, 60, 20);

    // Loop to continuously adjust motor power based on PID control
    while ((std::fabs(currentDistanceInDegrees) <= fabs(targetDistanceInDegrees) - 5) && !crossed180)
    {

        // Check if passed the target because max range is 0 to +-180 then it changes signs at 0 and 180 mark.
        if (std::round(currentNormHeading) * turnDegrees < 0)
        {
            crossed180 = true;
        }

        // Get current heading with offset conversion and normalize
        normCurrentHeading = normHeading(convertHeading(InertialSensor.heading(degrees), headingOffset));
        currentDistanceInDegrees = normCurrentHeading - startingNormHeading;

        Brain.Screen.printAt(10, 100, "Curr Head: %.2f", normCurrentHeading);
        Brain.Screen.printAt(10, 120, "Curr Dist: %.2f", currentDistanceInDegrees);
        Brain.Screen.printAt(10, 140, "Target: %.2f", turnDegrees);
        /*
            //Calculate angular drive motor speed in radians
            for (int i = 0; i < 3; i++) {
                //Get motor RPM and convert to radians per second
                leftMotorCMPerSecond[i] = leftMotor[i].velocity(velocityUnits::rpm) * (wheelCircumferenceCM / 60.0);
                rightMotorCMPerSecond[i] = rightMotor[i].velocity(velocityUnits::rpm) * (wheelCircumferenceCM / 60.0);

                leftMotorAngularRadians[i] = (2 * leftMotorCMPerSecond[i]) / trackWidth;
                rightMotorAngularRadians[i] = (2 * rightMotorCMPerSecond[i]) / trackWidth;

                //Add up all radians per second to prepare for calculating average motor radians per second
               // totalMotorRadiansPerSecond += std::fabs(motorRadiansPerSecondLeft[i]) + std::fabs(motorRadiansPerSecondRight[i]);
            }

           // double avgMotorRadianPerSecond =  totalMotorRadiansPerSecond / numberDriveMotor; // Calculate average motor radians per second

            // Get actual robot angular speed in radians using inertial sensor
            robotRadiansPerSecond = InertialSensor.gyroRate(axisType::zaxis, velocityUnits::dps) * (M_PI / 180.0);
        */

        // Get motor RPM with adjustment for each side
        for (int i = 0; i < 3; i++)
        {
            // leftMotorRPM[i] = fabs(leftMotor[i].velocity(velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            // rightMotorRPM[i] = fabs(rightMotor[i].velocity(velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            leftMotorRPM[i] = fabs(leftMotor[i].velocity(velocityUnits::rpm));
            rightMotorRPM[i] = fabs(rightMotor[i].velocity(velocityUnits::rpm));
        }

        // Get encoder RPM with both circumference and radius ratio adjustments
        double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double avgEncoderRPM = (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2;
        // double minSpeedVoltage = std::copysign((minSpeed * 0.01 * 12), normTargetHeading);

        // Calculate current distance in degrees
        // currentDistanceInDegrees = normalizeHeading(InertialSensor.heading()) - startingDegrees;
        // currentDistanceInDegrees = currentNormHeading - startingNormHeading;
        Brain.Screen.printAt(10, 160, "RightRPM[0]: %.2f", rightMotorRPM[0]);
        // Brain.Screen.printAt(10, 180, "RightVolt[0]: %.2f", motorVoltageRight[1]);
        Brain.Screen.printAt(10, 180, "LeftEncRPM: %.2f", leftEncoderRPM);
        Brain.Screen.printAt(10, 200, "RightEncRPM: %.2f", rightEncoderRPM);
        Brain.Screen.printAt(10, 220, "AvgEncRPM: %.2f", (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2);

        // avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2])/6;

        // Launch Phase
        // if (std::fabs(currentDistanceInDegrees) < (std::fabs(targetDistanceInDegrees) - breakDistanceInDegrees) && !accelCompleted && !turnCompleted) {
        if (fabs(currentDistanceInDegrees) < (fabs(targetDistanceInDegrees) - fabs(breakDistanceInDegrees)) && !accelCompleted && !decel)
        {
            for (int i = 0; i < 3; i++)
            {
                // Use leftLaunchControl if minLaunchPower threshold is met for the left side
                motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
                motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
                // motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorAngularRadians[i], robotRadiansPerSecond, accelFactorLaunch);      // get slip voltage
                // motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorAngularRadians[i], robotRadiansPerSecond, accelFactorLaunch);
            }

            averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;
            // averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;
            //    Brain.Screen.printAt(10, 20, "Launch Phase");

            //  if (std::fabs(robotRadiansPerSecond) >= maxRadiansPerSecond * (1 - percentSpeedLoss)){
            //      accelCompleted = true;
            //  }

            if (std::fabs(averageMotorVoltage) >= std::fabs(maxSpeedVoltage))
            {
                accelCompleted = true;
                Brain.Screen.printAt(10, 80, "Accel Completed");
            }

            Brain.Screen.printAt(10, 20, "Launch Phase");
            // Brain.Screen.printAt(10, 180, "Current Heading: %.2f", currentNormHeading);
            // Brain.Screen.printAt(10, 200, "Target Distance: %.2f", targetDistanceInDegrees);

            // Cruise Phase

            // testing cruise with traction control
        }
        else if (fabs(currentDistanceInDegrees) < (fabs(targetDistanceInDegrees) - fabs(breakDistanceInDegrees)) && accelCompleted)
        {
            // break;

            Brain.Screen.printAt(10, 20, "Cruise Phase"); // Else condition - specify actions here if the if condition is not met
            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = maxSpeedVoltage;
                motorVoltageRight[i] = maxSpeedVoltage;
            }

            // for (int i = 0; i < 3; i++) {
            //  Use leftLaunchControl if minLaunchPower threshold is met for the left side
            // motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorAngularRadians[i], robotRadiansPerSecond, accelFactorCruise);      // get slip voltage
            // motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorAngularRadians[i], robotRadiansPerSecond, accelFactorCruise);
            // }

            // Decel Phase

            // If declerating then go to ABS routine
        }
        else if (fabs(currentDistanceInDegrees) >= (fabs(targetDistanceInDegrees) - breakDistanceInDegrees) && decelCompleted == false)
        {
            // break;
            decel = true;
            Brain.Screen.printAt(10, 20, "Decel Phase");

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = 0;
                motorVoltageRight[i] = 0;
            }

            for (int i = 0; i < 3; i++)
            {
                // Getting brake mode from ABS controller
                leftBrakeMode[i] = ABSControllerLeft[i].ABSSpeedReduction(leftMotorRPM[i], leftEncoderRPM);
                rightBrakeMode[i] = ABSControllerRight[i].ABSSpeedReduction(rightMotorRPM[i], rightEncoderRPM);
            }

            // Pair 1: Leading wheels (left[0] and right[2])
            brakeType leadingPairMode = (leftBrakeMode[0] == brakeType::coast || rightBrakeMode[2] == brakeType::coast) ? brakeType::coast : brakeType::brake;
            leftMotor[0].stop(leadingPairMode);
            rightMotor[2].stop(leadingPairMode);

            // Pair 2: Middle wheels
            brakeType middlePairMode = (leftBrakeMode[1] == brakeType::coast || rightBrakeMode[1] == brakeType::coast) ? brakeType::coast : brakeType::brake;
            leftMotor[1].stop(middlePairMode);
            rightMotor[1].stop(middlePairMode);

            // Pair 3: Trailing wheels
            brakeType trailingPairMode = (leftBrakeMode[2] == brakeType::coast || rightBrakeMode[0] == brakeType::coast) ? brakeType::coast : brakeType::brake;
            leftMotor[2].stop(trailingPairMode);
            rightMotor[0].stop(trailingPairMode);

            /*
                // Stop the motors
               for (int i = 0; i < 3; i++) {
               leftMotor[i].stop(brake);
               rightMotor[i].stop(brake);
               }
           */

            //  decelCompleted = true;
            // If all drivetrain motors decel to min speed then change DecelCompleted State variable to true to start Approach Phase
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Detect if robot slowed down to target minimum speed
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }

            // Final Approach Phase
        }
        else if (decelCompleted == true)
        {
            // break;
            Brain.Screen.printAt(10, 20, "Approach Phase");
            Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted);

            // Calculate the PID output for distance control
            // double distanceCorrection = distancePID.calculate(targetDistance, currentDistance);

            // **Check if Distance Error is Within Target Zone and Update Stability Counter**
            // double distanceError = targetDistance - currentDistance;

            // Set the speed to minSpeed after the first stabilization
            //    distanceCorrection = (distanceCorrection > 0) ? minSpeedVoltage : -minSpeedVoltage;
            // double adjustedHeadingCorrection = headingCorrection * (avgEncoderSpeedRPM / maxRPM) * 2.0; //dynamically reduce heading correction at slower speed based on percentage of max speed

            // Else condition - specify actions here if the if condition is not met
            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = minSpeedVoltage;
                motorVoltageRight[i] = minSpeedVoltage;
            }
            Brain.Screen.printAt(10, 20, "Approach Phase");
        }

        // Pair 1: Leading wheels
        double lowerVoltageLeading = std::min(std::fabs(motorVoltageLeft[0]), std::fabs(motorVoltageRight[2]));
        motorVoltageLeft[0] = std::copysign(lowerVoltageLeading, motorVoltageLeft[0]);
        motorVoltageRight[2] = std::copysign(lowerVoltageLeading, motorVoltageRight[2]);

        // Pair 2: Middle wheels
        double lowerVoltageMiddle = std::min(std::fabs(motorVoltageLeft[1]), std::fabs(motorVoltageRight[1]));
        motorVoltageLeft[1] = std::copysign(lowerVoltageMiddle, motorVoltageLeft[1]);
        motorVoltageRight[1] = std::copysign(lowerVoltageMiddle, motorVoltageRight[1]);

        // Pair 3: Trailing wheels
        double lowerVoltageTrailing = std::min(std::fabs(motorVoltageLeft[2]), std::fabs(motorVoltageRight[0]));
        motorVoltageLeft[2] = std::copysign(lowerVoltageTrailing, motorVoltageLeft[2]);
        motorVoltageRight[0] = std::copysign(lowerVoltageTrailing, motorVoltageRight[0]);

        // Power Drive Motors

        // turnDirection = std::copysign(turnDirection, normTargetHeading);
        if (!decel == true || decelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
                rightMotor[i].spin(forward, -motorVoltageRight[i], voltageUnits::volt);
            }
        }
        // Brain.Screen.printAt(10, 20, "Rotation Sensor: %.2f degrees", rotationDegrees);
        // Brain.Screen.printAt(10, 40, "Current Distance: %.4f", currentDistance);
        // Brain.Screen.printAt(10, 80, "Break Distance: %.2f", (targetDistance - breakDistance));
        // Brain.Screen.printAt(10, 100, "Norm Heading: %.2f", normalizeHeading(InertialSensor.heading()));
        // Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotor[2].velocity(vex::velocityUnits::pct));
        // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotor[2].velocity(vex::velocityUnits::rpm));
        // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
        // Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
        // Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);

        vex::task::sleep(20);
    }

    // Stop all motors at end of routine after approach
    for (int i = 0; i < 3; i++)
    {
        // leftMotor[i].stop(brake);
        // rightMotor[2-i].stop(brake);
        leftMotor[i].stop(brake);
        rightMotor[2 - i].stop(brake);
    }
}

void turnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed)
{
    // Reset completion flags
    bool crossed180 = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = convertHeading(InertialSensor.heading(degrees), headingOffset);
    double headingError = normalizeHeading180(targetHeading - currentHeading);
    double currentDistanceInDegrees = headingError;

    Brain.Screen.printAt(10, 40, "Target Head: %.2f", targetHeading);
    Brain.Screen.printAt(10, 60, "Curr Heading: %.2f", convertEuclideanToVEX(currentHeading));

    // Convert % Speed input to voltage with max voltage of 12
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, headingError);
    double launchVoltage = std::copysign(5, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxDriveMotorRPM = (maxSpeed * .01) * absoluteMaxRPM;

    double percentSpeedLoss = 0.2;

    // Maximum slip threshold for spot turns before reducing power
    // Range: 0-1, where:
    // 0 = No slip allowed (most conservative)
    // 1 = Full slip allowed (most aggressive)
    // 0.25 = 25% slip tolerance for balanced control
    // may need to go above 25% given built in difference between encoder and wheel spin speed
    const double SLIP_THRESHOLD_TRACTION = 0.35; //somwewhere between 40 to 60 seems good, at least for 180 turns. 45 seems pretty good.
    const double SLIP_THRESHOLD_ABS = 0.35;

    double accelFactorLaunch = 1.4;
    double accelFactorCruise = 1.1;
    double encoderMotorScaleFactor = 0.857143; // 0.857143
    // double ABSLockThresholdSpotTurn = 0;
    // double totalMotorRadiansPerSecond = 0.0;
    // double minRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (minSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert minspeed to percentage first
    // double maxRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (maxSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert maxspeed to percentage first
    // double turnDirection = 1;
    double currentDegrees = 0;
    double currentNormHeading = 0;
    // double targetDriveVoltageLeft = 0;
    // double targetDriveVoltageRight = 0;
    // double motorRadiansPerSecondLeft[3];
    // double motorRadiansPerSecondRight[3];
    double leftMotorCMPerSecond[3];
    double rightMotorCMPerSecond[3];
    double leftMotorAngularRadians[3];
    double rightMotorAngularRadians[3];
    double leftMotorRPM[3] = {0, 0, 0};
    double rightMotorRPM[3] = {0, 0, 0};
    double robotRadiansPerSecond = 0;
    double averageMotorVoltage = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage}; // Initialize all elements to minimum launch speed
    vex::brakeType leftBrakeMode[3];
    vex::brakeType rightBrakeMode[3];
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    // double motorRadiansPerSecond[3] = {0, 0, 0};
    // double lowestMotorVoltage = 12;

    // MotionPhase currentPhase = READY;

    // Declaration for slip threshold
    ABSController ABSControllerLeft[3] = {
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS)};
    ABSController ABSControllerRight[3] = {
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS),
        ABSController(SLIP_THRESHOLD_ABS)};

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    tractionControl tractionControlRight[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION)};

    // launchControl (60, 60, 20);

    // Loop to continuously adjust motor power based on PID control
    while (std::abs(headingError) > 9) // Removed crossed180 check
    {
        currentHeading = convertHeading(InertialSensor.heading(degrees), headingOffset);
        headingError = normalizeHeading180(targetHeading - currentHeading);
        currentDistanceInDegrees = headingError;

        Brain.Screen.printAt(10, 100, "Curr Heading: %.2f", convertEuclideanToVEX(currentHeading));
        Brain.Screen.printAt(10, 120, "Curr Dist: %.2f", currentDistanceInDegrees);
        Brain.Screen.printAt(10, 140, "Target: %.2f", targetHeading);
        /*
            //Calculate angular drive motor speed in radians
            for (int i = 0; i < 3; i++) {
                //Get motor RPM and convert to radians per second
                leftMotorCMPerSecond[i] = leftMotor[i].velocity(velocityUnits::rpm) * (wheelCircumferenceCM / 60.0);
                rightMotorCMPerSecond[i] = rightMotor[i].velocity(velocityUnits::rpm) * (wheelCircumferenceCM / 60.0);

                leftMotorAngularRadians[i] = (2 * leftMotorCMPerSecond[i]) / trackWidth;
                rightMotorAngularRadians[i] = (2 * rightMotorCMPerSecond[i]) / trackWidth;

                //Add up all radians per second to prepare for calculating average motor radians per second
               // totalMotorRadiansPerSecond += std::fabs(motorRadiansPerSecondLeft[i]) + std::fabs(motorRadiansPerSecondRight[i]);
            }

           // double avgMotorRadianPerSecond =  totalMotorRadiansPerSecond / numberDriveMotor; // Calculate average motor radians per second

            // Get actual robot angular speed in radians using inertial sensor
            robotRadiansPerSecond = InertialSensor.gyroRate(axisType::zaxis, velocityUnits::dps) * (M_PI / 180.0);
        */

        // Get motor RPM with adjustment for each side
        for (int i = 0; i < 3; i++)
        {
            // leftMotorRPM[i] = fabs(leftMotor[i].velocity(velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            // rightMotorRPM[i] = fabs(rightMotor[i].velocity(velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            leftMotorRPM[i] = fabs(leftMotor[i].velocity(velocityUnits::rpm));
            rightMotorRPM[i] = fabs(rightMotor[i].velocity(velocityUnits::rpm));
        }

        // Get encoder RPM with both circumference and radius ratio adjustments
        double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
        double avgEncoderRPM = (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2;
        // double minSpeedVoltage = std::copysign((minSpeed * 0.01 * 12), normTargetHeading);

        // Calculate current distance in degrees
        // currentDistanceInDegrees = normalizeHeading(InertialSensor.heading()) - startingDegrees;
        // currentDistanceInDegrees = currentNormHeading - startingNormHeading;
        Brain.Screen.printAt(10, 160, "RightRPM[0]: %.2f", rightMotorRPM[0]);
        // Brain.Screen.printAt(10, 180, "RightVolt[0]: %.2f", motorVoltageRight[1]);
        Brain.Screen.printAt(10, 180, "LeftEncRPM: %.2f", leftEncoderRPM);
        Brain.Screen.printAt(10, 200, "RightEncRPM: %.2f", rightEncoderRPM);
        Brain.Screen.printAt(10, 220, "AvgEncRPM: %.2f", (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2);

        // avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2])/6;

        // Launch Phase
        // if (std::fabs(currentDistanceInDegrees) < (std::fabs(targetDistanceInDegrees) - breakDistanceInDegrees) && !accelCompleted && !turnCompleted) {
        if ((std::fabs(headingError) > fabs(breakDistanceInDegrees)) && !accelCompleted && !decel)
        {
            for (int i = 0; i < 3; i++)
            {
                // Use leftLaunchControl if minLaunchPower threshold is met for the left side
                motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
                motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorRPM[i], avgEncoderRPM, accelFactorLaunch);
                // motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorAngularRadians[i], robotRadiansPerSecond, accelFactorLaunch);      // get slip voltage
                // motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorAngularRadians[i], robotRadiansPerSecond, accelFactorLaunch);
            }

            averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;
            // averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;
            //    Brain.Screen.printAt(10, 20, "Launch Phase");

            //  if (std::fabs(robotRadiansPerSecond) >= maxRadiansPerSecond * (1 - percentSpeedLoss)){
            //      accelCompleted = true;
            //  }

            if (std::fabs(averageMotorVoltage) >= std::fabs(maxSpeedVoltage))
            {
                accelCompleted = true;
                Brain.Screen.printAt(10, 80, "Accel Completed");
            }

            Brain.Screen.printAt(10, 20, "Launch Phase");
            // Brain.Screen.printAt(10, 180, "Current Heading: %.2f", currentNormHeading);
            // Brain.Screen.printAt(10, 200, "Target Distance: %.2f", targetDistanceInDegrees);

            // Cruise Phase

            // testing cruise with traction control
        }
        else if ((std::abs(headingError) > fabs(breakDistanceInDegrees)) && accelCompleted)
        {
            // break;

            Brain.Screen.printAt(10, 20, "Cruise Phase"); // Else condition - specify actions here if the if condition is not met
            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = maxSpeedVoltage;
                motorVoltageRight[i] = maxSpeedVoltage;
            }

            // for (int i = 0; i < 3; i++) {
            //  Use leftLaunchControl if minLaunchPower threshold is met for the left side
            // motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorAngularRadians[i], robotRadiansPerSecond, accelFactorCruise);      // get slip voltage
            // motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorAngularRadians[i], robotRadiansPerSecond, accelFactorCruise);
            // }

            // Decel Phase

            // If declerating then go to ABS routine
        }
        else if ((std::abs(headingError) <= fabs(breakDistanceInDegrees)) && decelCompleted == false)
        {
            // break;
            decel = true;
            Brain.Screen.printAt(10, 20, "Decel Phase");

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = 0;
                motorVoltageRight[i] = 0;
            }

            for (int i = 0; i < 3; i++)
            {
                // Getting brake mode from ABS controller
                leftBrakeMode[i] = ABSControllerLeft[i].ABSSpeedReduction(leftMotorRPM[i], leftEncoderRPM);
                rightBrakeMode[i] = ABSControllerRight[i].ABSSpeedReduction(rightMotorRPM[i], rightEncoderRPM);
            }

            // Pair 1: Leading wheels (left[0] and right[2])
            brakeType leadingPairMode = (leftBrakeMode[0] == brakeType::coast || rightBrakeMode[2] == brakeType::coast) ? brakeType::coast : brakeType::brake;
            leftMotor[0].stop(leadingPairMode);
            rightMotor[2].stop(leadingPairMode);

            // Pair 2: Middle wheels
            brakeType middlePairMode = (leftBrakeMode[1] == brakeType::coast || rightBrakeMode[1] == brakeType::coast) ? brakeType::coast : brakeType::brake;
            leftMotor[1].stop(middlePairMode);
            rightMotor[1].stop(middlePairMode);

            // Pair 3: Trailing wheels
            brakeType trailingPairMode = (leftBrakeMode[2] == brakeType::coast || rightBrakeMode[0] == brakeType::coast) ? brakeType::coast : brakeType::brake;
            leftMotor[2].stop(trailingPairMode);
            rightMotor[0].stop(trailingPairMode);

            /*
                // Stop the motors
               for (int i = 0; i < 3; i++) {
               leftMotor[i].stop(brake);
               rightMotor[i].stop(brake);
               }
           */

            //  decelCompleted = true;
            // If all drivetrain motors decel to min speed then change DecelCompleted State variable to true to start Approach Phase
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Detect if robot slowed down to target minimum speed
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }

            // Final Approach Phase
        }
        else if (decelCompleted == true)
        {
            // break;
            Brain.Screen.printAt(10, 20, "Approach Phase");
            Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted);

            // Calculate the PID output for distance control
            // double distanceCorrection = distancePID.calculate(targetDistance, currentDistance);

            // **Check if Distance Error is Within Target Zone and Update Stability Counter**
            // double distanceError = targetDistance - currentDistance;

            // Set the speed to minSpeed after the first stabilization
            //    distanceCorrection = (distanceCorrection > 0) ? minSpeedVoltage : -minSpeedVoltage;
            // double adjustedHeadingCorrection = headingCorrection * (avgEncoderSpeedRPM / maxRPM) * 2.0; //dynamically reduce heading correction at slower speed based on percentage of max speed

            // Else condition - specify actions here if the if condition is not met
            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = minSpeedVoltage;
                motorVoltageRight[i] = minSpeedVoltage;
            }
            Brain.Screen.printAt(10, 20, "Approach Phase");
        }

        // Pair 1: Leading wheels
        double lowerVoltageLeading = std::min(std::fabs(motorVoltageLeft[0]), std::fabs(motorVoltageRight[2]));
        motorVoltageLeft[0] = std::copysign(lowerVoltageLeading, motorVoltageLeft[0]);
        motorVoltageRight[2] = std::copysign(lowerVoltageLeading, motorVoltageRight[2]);

        // Pair 2: Middle wheels
        double lowerVoltageMiddle = std::min(std::fabs(motorVoltageLeft[1]), std::fabs(motorVoltageRight[1]));
        motorVoltageLeft[1] = std::copysign(lowerVoltageMiddle, motorVoltageLeft[1]);
        motorVoltageRight[1] = std::copysign(lowerVoltageMiddle, motorVoltageRight[1]);

        // Pair 3: Trailing wheels
        double lowerVoltageTrailing = std::min(std::fabs(motorVoltageLeft[2]), std::fabs(motorVoltageRight[0]));
        motorVoltageLeft[2] = std::copysign(lowerVoltageTrailing, motorVoltageLeft[2]);
        motorVoltageRight[0] = std::copysign(lowerVoltageTrailing, motorVoltageRight[0]);

        // Power Drive Motors

        // turnDirection = std::copysign(turnDirection, normTargetHeading);
        if (!decel == true || decelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
                rightMotor[i].spin(forward, -motorVoltageRight[i], voltageUnits::volt);
            }
        }

        vex::task::sleep(10);
    }

    // Stop all motors at end of routine after approach
    for (int i = 0; i < 3; i++)
    {
        // leftMotor[i].stop(brake);
        // rightMotor[2-i].stop(brake);
        leftMotor[i].stop(brake);
        rightMotor[2 - i].stop(brake);
    }
}

// Traction Control Constructor implementation
tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold)
    : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {}

// Method to determine and adjust motor voltage based on wheel slip
double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, double robotSpeed, double accelFactor)
{

    // Prevent division by zero in slip calculation by ensuring minimum wheel speed
    if (fabs(wheelSpeed) == 0)
    {
        wheelSpeed = 0.1;
    }

    // Calculate slip ratio = (Wheel Speed - Robot Speed) / Wheel Speed or Robot Speed, which ever is higher to better measure differential
    // This gives us percentage of wheel spin relative to actual travel speed
    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed); // Formula is in utls.cpp

    // If slip exceeds threshold, reduce power to regain traction
    // If slip is under control, gradually increase available power
    if (slipRatio > slipThreshold)
    {
        motorVoltage = motorVoltage / accelFactor; // Reduce power when slipping
    }
    else
    {
        motorVoltage = motorVoltage * accelFactor; // Increase power when grip is good
    }

    // Clamp voltage between the min and max while keeping the sign of maxSpeedVoltage
    // This ensures we stay within safe operating voltage range while preserving direction
    motorVoltage = std::copysign(std::max(std::fabs(minSpeedVoltage), std::min(std::fabs(motorVoltage), std::fabs(maxSpeedVoltage))), motorVoltage);

    return motorVoltage;
}

ABSController::ABSController(double lockThreshold) : ABSLockThreshold(lockThreshold) {}

vex::brakeType ABSController::ABSSpeedReduction(double wheelSpeed, double robotSpeed)
{
    if (fabs(wheelSpeed) == 0)
    {
        wheelSpeed = 0.1;
    }

    double slipRatio = calculateSlipRatio(wheelSpeed, robotSpeed); // Formula is in utls.cpp

    if (slipRatio > ABSLockThreshold)
    {
        return vex::coast;
    }
    else
    {
        return vex::brake;
    }
}

void straight(double targetDistance,
              double breakDistance,
              double minSpeed,
              double targetHeading,
              double kp_heading,
              double ki_heading,
              double kd_heading,
              double accelHeadingScaling,
              double decelHeadingScaling,
              double approachHeadingScaling,
              double maxSpeed)
{

    // Add timer for acceleration phase
    vex::timer accelTimer;
    double accelTime = 0;

    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset(); // Remove extra blank line after this

    // Motion Parameters
    double currentDistance = 0;
    double headingDirection = (targetDistance > 0) ? 1.0 : -1.0;

    // Target Speeds & Voltages
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(5, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), targetDistance);
    static constexpr double VOLTAGE_TOLERANCE = 0.1;

    // RPM Parameters
    double percentRPMLoss = 0.25;
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxDriveMotorRPM = (maxSpeed * .01) * absoluteMaxRPM;

    // Motor Arrays
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftMotorRPM[3] = {0, 0, 0};
    double rightMotorRPM[3] = {0, 0, 0};

    // Traction Control Parameters
    // slipThreshold: 0-1 range (0 = no slip allowed, 1 = full slip allowed, .15-.25 = optimal slip)
    double slipThresholdTraction = 0.30;
    double slipThresholdABS = 0.25;
    // double accelFactorLaunch = 1.4; //good starting launch acceleration factor
    double accelFactorLaunch = 1.15; // test, temporary

    // PID and Heading Control
    // Get initial heading when straight movement starts
    // PID and Heading Control
    double avgMotorVoltage = 0; // Used for phase transition checking
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    // Declaration for slip threshold
    ABSController ABSControllerLeft[3] = {
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS)};
    ABSController ABSControllerRight[3] = {
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS)};
    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction)};

    tractionControl tractionControlRight[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction)};

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 4)
    {

        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        /*
        // Distance calculation debug
        Brain.Screen.clearScreen();  // Clear previous prints
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("L/R deg: %.1f/%.1f",
            passiveEncoderLeft.position(degrees),
            passiveEncoderRight.position(degrees));
        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("Dist/Target: %.1f/%.1f", currentDistance, targetDistance);

        Brain.Screen.setCursor(4,1);
        Brain.Screen.print("Launch/Cruise/Decel: %d/%d/%d",
            (std::fabs(currentDistance) < (std::fabs(targetDistance) - breakDistance) && !accelCompleted && !decel),
            (std::fabs(currentDistance) < (std::fabs(targetDistance) - breakDistance) && accelCompleted == true),
            (std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance));
        */

        // Calculate the heading correction using the PID controller
        // Calculate the heading correction using normalized error
        double currentHeading = InertialSensor.heading(degrees);
        double headingError = normalizeHeading180(targetHeading - currentHeading);
        double headingCorrection = headingPID.calculate(0, headingError);
        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;
        // double adjustedHeadingCorrection = headingCorrection * avgEncoderRPM / absoluteMaxRPM; //dynamically reduce heading correction at slower speed based on percentage of max speed
        /*
                //Quadratics scaling factor for PID
                double scaleFactorPower = 2;
                double speedRatio = std::min(1.0, std::fabs(avgEncoderRPM / maxDriveMotorRPM)); //caps max speed ratio at 1:1
                double scaleFactor = std::pow(speedRatio, scaleFactorPower);
                double adjustedHeadingCorrection = headingCorrection * scaleFactor;
        */

        for (int i = 0; i < 3; i++)
        {
            // Use leftLaunchControl if minLaunchPower threshold is met for the left side
            // Get motor speed in RPM
            leftMotorRPM[i] = leftMotor[i].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            rightMotorRPM[i] = rightMotor[i].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        }

        /*
            Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Distance: %.1f Target: %.1f Break: %.1f",
            std::fabs(currentDistance), std::fabs(targetDistance), breakDistance);

        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("Decel/Accel: %d/%d", decel, accelCompleted);

        Brain.Screen.setCursor(3,1);
        Brain.Screen.print("Dist Check: %d",
            std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance);
        */

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        { // will keep going until acceleration is complete and not in decel
            Brain.Screen.printAt(10, 20, "Launch Phase");
            for (int i = 0; i < 3; i++)
            {

                // Print initial values
                // Brain.Screen.printAt(10, 80, "Init L: %.2f, R: %.2f", motorVoltageLeft[i], motorVoltageRight[i]);

                // Call traction cotrol class and get adjusted motor voltage
                motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorRPM[i], avgEncoderRPM, accelFactorLaunch) + (headingCorrection * accelHeadingScaling); // get slip voltage and Adjust for heading correction
                motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorRPM[i], avgEncoderRPM, accelFactorLaunch) - (headingCorrection * accelHeadingScaling);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);

                double maxMotorRPM = std::max(maxMotorRPM, fabs(rightMotorRPM[i]));
                double maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));
            }

            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {

                //  Brain.Screen.setCursor(12,1);
                // Brain.Screen.print("Raw RPM L/R: %.1f/%.1f", leftMotor[0].velocity(velocityUnits::rpm), rightMotor[0].velocity(velocityUnits::rpm));

                accelCompleted = true;
                /*
                        // Stop all motors at end of routine after approach
                        for (int i = 0; i < 3; i++) {
                            motorVoltageLeft[i] = 0;
                            motorVoltageRight[i] = 0;
                            leftMotor[i].stop(brake);
                            rightMotor[i].stop(brake);
                        }
                  */
            }

            // Cruise Phase
        }
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            // break;
            Brain.Screen.printAt(10, 20, "Cruise Phase"); // Else condition - specify actions here if the if condition is not met

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = maxSpeedVoltage + (headingCorrection);
                motorVoltageRight[i] = maxSpeedVoltage - (headingCorrection);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);
            }

            /*
                Brain.Screen.setCursor(1,1);
            Brain.Screen.print("Distance: %.1f Target: %.1f Break: %.1f",
                std::fabs(currentDistance), std::fabs(targetDistance), breakDistance);

            Brain.Screen.setCursor(2,1);
            Brain.Screen.print("Accel/Decel: %d/%d", accelCompleted, decel);

            Brain.Screen.setCursor(3,1);
            Brain.Screen.print("Dist Check: %d",
                std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance);

            */

            // Decel Phase
            // If declerating then go to ABS routine
        }
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            // break;
            // Sets motorvoltage to zero so it defaults to brake when it first enters then ABS takes over
            Brain.Screen.setCursor(5, 1);
            Brain.Screen.print("In Decel");

            if (!decel)
            {
                for (int i = 0; i < 3; i++)
                {
                    // Example action: Set motor voltage to target voltage directly
                    motorVoltageLeft[i] = 0;
                    motorVoltageRight[i] = 0;
                }
            }
            Brain.Screen.printAt(10, 20, "Decel Phase");
            decel = true;

            for (int i = 0; i < 3; i++)
            {

                // Left Side
                vex::brakeType leftBrakeMode = ABSControllerLeft[i].ABSSpeedReduction(leftMotorRPM[i], avgEncoderRPM);    // Direct brake mode
                vex::brakeType rightBrakeMode = ABSControllerRight[i].ABSSpeedReduction(rightMotorRPM[i], avgEncoderRPM); // Direct brake mode //Adding heading PID correction during coast phase and reduce regen braking by adding min speed (smaller the voltage, the greater the resistance).

                // Adding PID correction during coast modes to keep robot going straight
                if (leftBrakeMode == brakeType::coast)
                {
                    motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * decelHeadingScaling * headingDirection); // Add heading correction and user adjustment factor. Min speed added to reduce regen brake.
                    Brain.Screen.printAt(10, 40, "Decel Heading Correction");
                }
                else
                { // If not coasting, then it must be braking
                    leftMotor[i].stop(leftBrakeMode);
                    motorVoltageLeft[i] = 0;
                }

                if (rightBrakeMode == brakeType::coast)
                {
                    motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * decelHeadingScaling * headingDirection);
                    // Brain.Screen.printAt(10, 40, "Decel Heading Correction");
                }
                else
                { // If not coasting, then it must be braking
                    rightMotor[i].stop(rightBrakeMode);
                    motorVoltageRight[i] = 0;
                }
                // Brain.Screen.printAt(10, 140, "motorVoltageLeft: %d", static_cast<int>(motorVoltageLeft[2]));
                // Brain.Screen.printAt(10, 160, "motorVoltageRight: %d", static_cast<int>(motorVoltageRight[2]));

                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);

                // Right Side

                // if (rightResult.brakeMode == brakeType::coast) {
                //     motorVoltageRight[i] += minSpeedVoltage - (adjustedHeadingCorrection * decelHeadingScaling); // Add heading correction and user adjustment factor. Min speed added to reduce regen brake.
                // }
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Detect if robot slowed down to target minimum speed
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;

                // if (fabs(avgEncoderRPM) <= fabs(minDriveMotorRPM)) {
                //     decelCompleted = true;
            }

            // Final Approach Phase
        }
        else if (decelCompleted == true)
        {
            // break;
            Brain.Screen.printAt(10, 20, "Approach Phase");
            Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted);

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);
            }
        }

        // Power Drive Motors

        // turnDirection = std::copysign(turnDirection, normTargetHeading);
        // if (!decel == true || decelCompleted == true)
        //{
        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
        }
        //}
        double avgMotorRPM = (leftMotorRPM[0] + leftMotorRPM[1] + leftMotorRPM[2] + rightMotorRPM[0] + rightMotorRPM[1] + rightMotorRPM[2]) / 6;
        //  Brain.Screen.printAt(10, 60, "Motor: %.2f, Robot L: %.2f, Robot R: %.2f", leftMotorRPM[1],leftEncoderRPM, rightEncoderRPM);
        // Brain.Screen.printAt(10, 60, "Left Encoder: %.2f degrees", leftEncoderRPM);
        // Brain.Screen.printAt(10, 80, "Right Encoder: %.2f", rightEncoderRPM);
        // Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotor[2].velocity(vex::velocityUnits::pct));
        // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotor[2].velocity(vex::velocityUnits::rpm));
        // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
        // Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
        // Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);

        vex::task::sleep(20);
    }

    // Stop all motors at end of routine after approach
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].stop(brake);
        rightMotor[i].stop(brake);
    }

    /*
        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
        // Debug print: Stopping motors
        Brain.Screen.clearLine(8);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Distance Complete");
        Brain.Screen.print("Current Distance: %.2f", currentDistance);
    */
}

void arcTurn(double targetDistance,
             double breakDistance,
             double minSpeed,
             double maxSpeed,
             double turnRadius, // Radius of turn in cm
             bool turnLeft)
{ // true for left turn, false for right

    // Calculate speed ratios for inner and outer wheels based on turn radius
    // Inner wheel travels less distance than outer wheel
    double innerRatio = (turnRadius - (TRACK_WIDTH / 2)) / turnRadius;
    double outerRatio = (turnRadius + (TRACK_WIDTH / 2)) / turnRadius;

    // Reset encoders
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    double currentDistance = 0;

    // Calculate max/min voltages for each wheel
    double maxVoltage = maxSpeed * 0.01 * absoluteMaxVoltage;
    double minVoltage = minSpeed * 0.01 * absoluteMaxVoltage;

    // Initial voltages start at minimum
    double innerVoltage = minVoltage * innerRatio;
    double outerVoltage = minVoltage * outerRatio;

    // Target max voltages
    double innerMaxVoltage = maxVoltage * innerRatio;
    double outerMaxVoltage = maxVoltage * outerRatio;

    while (std::fabs(currentDistance) <= fabs(targetDistance))
    {
        // Update current distance (average of wheels)
        currentDistance = ((passiveEncoderLeft.position(degrees) +
                            passiveEncoderRight.position(degrees)) /
                           2.0 / 360.0) *
                          encoderWheelCircumferenceCM;

        // Acceleration phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance))
        {
            // Gradually increase voltage while maintaining ratio
            innerVoltage = std::min(innerMaxVoltage, innerVoltage + 0.1);
            outerVoltage = std::min(outerMaxVoltage, outerVoltage + 0.1);
        }
        // Deceleration phase
        else
        {
            // Gradually decrease voltage while maintaining ratio
            innerVoltage = std::max(minVoltage * innerRatio, innerVoltage - 0.1);
            outerVoltage = std::max(minVoltage * outerRatio, outerVoltage - 0.1);
        }

        // Apply voltages based on turn direction
        if (turnLeft)
        {
            // Left turn - left wheel is inner wheel
            for (int i = 0; i < 3; i++)
            {
                leftMotor[i].spin(forward, innerVoltage, voltageUnits::volt);
                rightMotor[i].spin(forward, outerVoltage, voltageUnits::volt);
            }
        }
        else
        {
            // Right turn - right wheel is inner wheel
            for (int i = 0; i < 3; i++)
            {
                leftMotor[i].spin(forward, outerVoltage, voltageUnits::volt);
                rightMotor[i].spin(forward, innerVoltage, voltageUnits::volt);
            }
        }

        vex::task::sleep(20);
    }

    // Stop motors at end
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].stop(brake);
        rightMotor[i].stop(brake);
    }
}

void straightOdometry(double targetDistance,
                      double breakDistance,
                      double minSpeed,
                      double targetHeading,
                      double kp_heading,
                      double ki_heading,
                      double kd_heading,
                      double accelHeadingScaling,
                      double decelHeadingScaling,
                      double approachHeadingScaling,
                      double maxSpeed)
{

    // Add timer for acceleration phase
    vex::timer accelTimer;
    double accelTime = 0;

    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset(); // Remove extra blank line after this

    // Motion Parameters
    double currentDistance = 0;
    //double headingDirection = (targetDistance > 0) ? 1.0 : -1.0;

    // Target Speeds & Voltages
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(5, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), targetDistance);
    static constexpr double VOLTAGE_TOLERANCE = 0.1;

    // RPM Parameters
    double percentRPMLoss = 0.25;
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxDriveMotorRPM = (maxSpeed * .01) * absoluteMaxRPM;
    double maxMotorRPM = 0;
    double maxEncoderRPM = 0;
    // Motor Arrays
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftMotorRPM[3] = {0, 0, 0};
    double rightMotorRPM[3] = {0, 0, 0};

    // Traction Control Parameters
    // slipThreshold: 0-1 range (0 = no slip allowed, 1 = full slip allowed, .15-.25 = optimal slip)
    double slipThresholdTraction = 0.35;
    double slipThresholdABS = 0.25;
    // double accelFactorLaunch = 1.4; //good starting launch acceleration factor
    double accelFactorLaunch = 1.25; // test, temporary

    // PID and Heading Control

    // double normTargetHeading = normHeading(targetHeading);
    double avgMotorVoltage = 0; // Used for phase transition checking
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

    // Declaration for slip threshold
    ABSController ABSControllerLeft[3] = {
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS)};
    ABSController ABSControllerRight[3] = {
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS),
        ABSController(slipThresholdABS)};
    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                              tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction)};

    tractionControl tractionControlRight[3] = {tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction),
                                               tractionControl(minLaunchSpeedVoltage, maxSpeedVoltage, slipThresholdTraction)};

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 3)
    {

        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        /*
        // Distance calculation debug
        Brain.Screen.clearScreen();  // Clear previous prints
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("L/R deg: %.1f/%.1f",
            passiveEncoderLeft.position(degrees),
            passiveEncoderRight.position(degrees));
        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("Dist/Target: %.1f/%.1f", currentDistance, targetDistance);

        Brain.Screen.setCursor(4,1);
        Brain.Screen.print("Launch/Cruise/Decel: %d/%d/%d",
            (std::fabs(currentDistance) < (std::fabs(targetDistance) - breakDistance) && !accelCompleted && !decel),
            (std::fabs(currentDistance) < (std::fabs(targetDistance) - breakDistance) && accelCompleted == true),
            (std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance));
        */

        // Calculate the heading correction using the PID controller
        // double headingError = getHeadingError360(targetHeading, InertialSensor.heading());
        // Calculate the heading correction using the PID controller with normalization
        // Calculate the heading correction using normalized error
        double currentHeading = convertHeading(InertialSensor.heading(degrees), headingOffset);
        double headingError = normalizeHeading180(targetHeading - currentHeading);
        double headingCorrection = headingPID.calculate(headingError,0);

                // Add these debug prints
        Brain.Screen.printAt(10, 160, "Target: %.1f", targetHeading);
        Brain.Screen.printAt(10, 180, "Current: %.1f", currentHeading);
        Brain.Screen.printAt(10, 200, "Error: %.1f", headingError);
        Brain.Screen.printAt(10, 220, "Correction: %.1f", headingCorrection);

        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;
        // double adjustedHeadingCorrection = headingCorrection * avgEncoderRPM / absoluteMaxRPM; //dynamically reduce heading correction at slower speed based on percentage of max speed
        /*
                //Quadratics scaling factor for PID
                double scaleFactorPower = 2;
                double speedRatio = std::min(1.0, std::fabs(avgEncoderRPM / maxDriveMotorRPM)); //caps max speed ratio at 1:1
                double scaleFactor = std::pow(speedRatio, scaleFactorPower);
                double adjustedHeadingCorrection = headingCorrection * scaleFactor;
        */

        for (int i = 0; i < 3; i++)
        {
            // Use leftLaunchControl if minLaunchPower threshold is met for the left side
            // Get motor speed in RPM
            leftMotorRPM[i] = leftMotor[i].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            rightMotorRPM[i] = rightMotor[i].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        }

        /*
            Brain.Screen.setCursor(1,1);
        Brain.Screen.print("Distance: %.1f Target: %.1f Break: %.1f",
            std::fabs(currentDistance), std::fabs(targetDistance), breakDistance);

        Brain.Screen.setCursor(2,1);
        Brain.Screen.print("Decel/Accel: %d/%d", decel, accelCompleted);

        Brain.Screen.setCursor(3,1);
        Brain.Screen.print("Dist Check: %d",
            std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance);
        */

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        { // will keep going until acceleration is complete and not in decel
            Brain.Screen.printAt(10, 20, "Launch Phase");
            for (int i = 0; i < 3; i++)
            {

                // Print initial values
                // Brain.Screen.printAt(10, 80, "Init L: %.2f, R: %.2f", motorVoltageLeft[i], motorVoltageRight[i]);

                // Call traction cotrol class and get adjusted motor voltage
                motorVoltageLeft[i] = tractionControlLeft[i].tractionControlSpeed(motorVoltageLeft[i], leftMotorRPM[i], avgEncoderRPM, accelFactorLaunch) + (headingCorrection * accelHeadingScaling); // get slip voltage and Adjust for heading correction
                motorVoltageRight[i] = tractionControlRight[i].tractionControlSpeed(motorVoltageRight[i], rightMotorRPM[i], avgEncoderRPM, accelFactorLaunch) - (headingCorrection * accelHeadingScaling);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);

                maxMotorRPM = std::max(maxMotorRPM, fabs(rightMotorRPM[i]));
                maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));
            }

            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {

                //  Brain.Screen.setCursor(12,1);
                // Brain.Screen.print("Raw RPM L/R: %.1f/%.1f", leftMotor[0].velocity(velocityUnits::rpm), rightMotor[0].velocity(velocityUnits::rpm));

                accelCompleted = true;
                /*
                        // Stop all motors at end of routine after approach
                        for (int i = 0; i < 3; i++) {
                            motorVoltageLeft[i] = 0;
                            motorVoltageRight[i] = 0;
                            leftMotor[i].stop(brake);
                            rightMotor[i].stop(brake);
                        }
                  */
            }

            // Cruise Phase
        }
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            // break;
            Brain.Screen.printAt(10, 20, "Cruise Phase"); // Else condition - specify actions here if the if condition is not met

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = maxSpeedVoltage + (headingCorrection);
                motorVoltageRight[i] = maxSpeedVoltage - (headingCorrection);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);
            }

            /*
                Brain.Screen.setCursor(1,1);
            Brain.Screen.print("Distance: %.1f Target: %.1f Break: %.1f",
                std::fabs(currentDistance), std::fabs(targetDistance), breakDistance);

            Brain.Screen.setCursor(2,1);
            Brain.Screen.print("Accel/Decel: %d/%d", accelCompleted, decel);

            Brain.Screen.setCursor(3,1);
            Brain.Screen.print("Dist Check: %d",
                std::fabs(currentDistance) >= std::fabs(targetDistance) - breakDistance);

            */

            // Decel Phase
            // If declerating then go to ABS routine
        }
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            // break;
            // Sets motorvoltage to zero so it defaults to brake when it first enters then ABS takes over
            Brain.Screen.setCursor(5, 1);
            Brain.Screen.print("In Decel");

            if (!decel)
            {
                for (int i = 0; i < 3; i++)
                {
                    // Example action: Set motor voltage to target voltage directly
                    motorVoltageLeft[i] = 0;
                    motorVoltageRight[i] = 0;
                }
            }
            Brain.Screen.printAt(10, 20, "Decel Phase");
            decel = true;

            for (int i = 0; i < 3; i++)
            {

                // Left Side
                vex::brakeType leftBrakeMode = ABSControllerLeft[i].ABSSpeedReduction(leftMotorRPM[i], avgEncoderRPM);    // Direct brake mode
                vex::brakeType rightBrakeMode = ABSControllerRight[i].ABSSpeedReduction(rightMotorRPM[i], avgEncoderRPM); // Direct brake mode //Adding heading PID correction during coast phase and reduce regen braking by adding min speed (smaller the voltage, the greater the resistance).

                // Adding PID correction during coast modes to keep robot going straight
                if (leftBrakeMode == brakeType::coast)
                {
                    motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * decelHeadingScaling); // Add heading correction and user adjustment factor. Min speed added to reduce regen brake.
                    Brain.Screen.printAt(10, 40, "Decel Heading Correction");
                }
                else
                { // If not coasting, then it must be braking
                    leftMotor[i].stop(leftBrakeMode);
                    motorVoltageLeft[i] = 0;
                }

                if (rightBrakeMode == brakeType::coast)
                {
                    motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * decelHeadingScaling);
                    // Brain.Screen.printAt(10, 40, "Decel Heading Correction");
                }
                else
                { // If not coasting, then it must be braking
                    rightMotor[i].stop(rightBrakeMode);
                    motorVoltageRight[i] = 0;
                }
                // Brain.Screen.printAt(10, 140, "motorVoltageLeft: %d", static_cast<int>(motorVoltageLeft[2]));
                // Brain.Screen.printAt(10, 160, "motorVoltageRight: %d", static_cast<int>(motorVoltageRight[2]));

                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);

                // Right Side

                // if (rightResult.brakeMode == brakeType::coast) {
                //     motorVoltageRight[i] += minSpeedVoltage - (adjustedHeadingCorrection * decelHeadingScaling); // Add heading correction and user adjustment factor. Min speed added to reduce regen brake.
                // }
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            // Detect if robot slowed down to target minimum speed
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;

                // if (fabs(avgEncoderRPM) <= fabs(minDriveMotorRPM)) {
                //     decelCompleted = true;
            }
            // test

            // Final Approach Phase
        }
        else if (decelCompleted == true)
        {
            // break;
            Brain.Screen.printAt(10, 20, "Approach Phase");
            Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted);

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);
            }
        }

        // Power Drive Motors

        // turnDirection = std::copysign(turnDirection, normTargetHeading);
        // if (!decel == true || decelCompleted == true)
        //{
        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
        }
        //}
        double avgMotorRPM = (leftMotorRPM[0] + leftMotorRPM[1] + leftMotorRPM[2] + rightMotorRPM[0] + rightMotorRPM[1] + rightMotorRPM[2]) / 6;
        //  Brain.Screen.printAt(10, 60, "Motor: %.2f, Robot L: %.2f, Robot R: %.2f", leftMotorRPM[1],leftEncoderRPM, rightEncoderRPM);
        // Brain.Screen.printAt(10, 60, "Left Encoder: %.2f degrees", leftEncoderRPM);
        // Brain.Screen.printAt(10, 80, "Right Encoder: %.2f", rightEncoderRPM);
        // Brain.Screen.printAt(10, 120, "Lt Motor Speed: %.2f", leftMotor[2].velocity(vex::velocityUnits::pct));
        // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotor[2].velocity(vex::velocityUnits::rpm));
        // Brain.Screen.printAt(10, 140, "Rt Motor Speed: %.2f", rightMotorSpeed);
        // Brain.Screen.printAt(10, 170, "Max Speed Voltage: %.2f", maxSpeedVoltage);
        // Brain.Screen.printAt(10, 190, "Min Speed Voltage: %.2f", minSpeedVoltage);

        vex::task::sleep(10);
    }

    // Stop all motors at end of routine after approach
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].stop(brake);
        rightMotor[i].stop(brake);
    }

    /*
        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
        // Debug print: Stopping motors
        Brain.Screen.clearLine(8);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Distance Complete");
        Brain.Screen.print("Current Distance: %.2f", currentDistance);
    */
}

//Forwards wrapper to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void forwardMP(double targetDistance,
                     double breakDistance, 
                     double minSpeed,
                     double cartesianAngle,
                     double kp_heading, 
                     double ki_heading,
                     double kd_heading, 
                     double accelHeadingScaling,
                     double decelHeadingScaling, 
                     double approachHeadingScaling,
                     double maxSpeed) {
    // Convert from Cartesian CCW to VEX CW
    double vexAngle = convertEuclideanToVEX(cartesianAngle);
    straightOdometry(targetDistance, breakDistance, minSpeed, vexAngle,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
}

//Backeward wrapper to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void backwardMP(double targetDistance,
                      double breakDistance, 
                      double minSpeed,
                      double cartesianAngle,
                      double kp_heading, 
                      double ki_heading,
                      double kd_heading, 
                      double accelHeadingScaling,
                      double decelHeadingScaling, 
                      double approachHeadingScaling,
                      double maxSpeed) {
    // Convert from Cartesian CCW to VEX CW
    double vexAngle = convertEuclideanToVEX(cartesianAngle);
    // Force negative distance for backward movement
    targetDistance = -std::fabs(targetDistance);
    straightOdometry(targetDistance, breakDistance, minSpeed, vexAngle,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
}

//Turn left to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void leftMP(double targetAngle, double breakDistance, double minSpeed, double maxSpeed) {
    // Convert from Euclidean CCW to VEX CW angle
    double vexAngle = convertEuclideanToVEX(targetAngle);
    
    // Pass through to turnOdometry with converted angle
    turnOdometry(vexAngle, breakDistance, minSpeed, maxSpeed);
}

//Turn right to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void rightMP(double targetAngle, double breakDistance, double minSpeed, double maxSpeed) {
    // Convert from Euclidean CCW to VEX CW angle
    double vexAngle = convertEuclideanToVEX(targetAngle);
    
    // Get current heading and error
    double currentHeading = convertHeading(InertialSensor.heading(degrees), headingOffset);
    double headingError = normalizeHeading180(vexAngle - currentHeading);
    
    // Round to nearest degree
    headingError = round(headingError);
    
    // Force CW direction only for 180 turns
    if (fabs(headingError) == 180) {
        maxSpeed = -abs(maxSpeed);
    }
    
    
    turnOdometry(vexAngle, breakDistance, minSpeed, maxSpeed);
}