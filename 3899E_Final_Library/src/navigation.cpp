#include "navigation.h"
#include "robot_config.h"
#include "vex.h"
#include "utils.h"
#include "pid.h" // Ensure this line is included
#include <cmath>
#include <algorithm>
#include <iterator>
#include <cstring> // Include the cstring library for strcmp
#include <atomic>
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


void turnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed)
{
    // Reset completion flags
    
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    int completeRotations = (int)(currentHeading / 360.0);  
    double targetRotationHeading = targetHeading + (completeRotations * 360.0);
    double headingError = targetRotationHeading - currentHeading;
    //double currentDistanceInDegrees = headingError;

    Brain.Screen.printAt(10, 40, "Target Head: %.2f", targetHeading);
    Brain.Screen.printAt(10, 100, "Curr Rotation: %.2f", currentHeading);

    // Convert % Speed input to voltage with max voltage of 12
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, headingError);
    double launchVoltage = std::copysign(4, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;

    // Maximum slip threshold for spot turns before reducing power
    // Range: 0-1, where:
    // 0 = No slip allowed (most conservative)
    // 1 = Full slip allowed (most aggressive)
    // 0.25 = 25% slip tolerance for balanced control
    // may need to go above 25% given built in difference between encoder and wheel spin speed
    const double TURN_ACCEL_FACTOR_LAUNCH = 1.2;
    const double SLIP_THRESHOLD_TRACTION = 10; //somwewhere between 40 to 60 seems good, at least for 180 turns. 45 seems pretty good.
    const double SLIP_THRESHOLD_ABS = 100;
    const double EXIT_TOLERANCE_DEGREES = 4;
      
    double averageMotorVoltage = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage}; // Initialize all elements to minimum launch speed
 
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    // Declaration for slip threshold
    ABSController ABSControllerLeft(SLIP_THRESHOLD_ABS);
    ABSController ABSControllerRight(SLIP_THRESHOLD_ABS);

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Loop to continuously adjust motor power 
    while ((maxSpeedVoltage > 0 && currentHeading <= targetRotationHeading - EXIT_TOLERANCE_DEGREES) || 
       (maxSpeedVoltage < 0 && currentHeading >= targetRotationHeading + EXIT_TOLERANCE_DEGREES)) 
       {
        currentHeading = InertialSensor.rotation(degrees) + headingOffset;
        headingError = targetRotationHeading - currentHeading;
        //currentDistanceInDegrees = headingError;
        
        Brain.Screen.printAt(10, 100, "Curr Rotation: %.2f", currentHeading);
        //Brain.Screen.printAt(10, 120, "Curr Dist: %.2f", currentDistanceInDegrees);
        Brain.Screen.printAt(10, 140, "Target: %.2f", targetHeading);
       
        // Get motor RPM with adjustment for each side
        double leftMotorRPM = fabs(leftMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = fabs(rightMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;

        // Get encoder RPM with both circumference and radius ratio adjustments
        double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                 (encoderWheelCircumferenceCM / wheelCircumferenceCM);

        // Average the encoder values
        double averageEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2.0;                         
        // double minSpeedVoltage = std::copysign((minSpeed * 0.01 * 12), normTargetHeading);

        // Calculate current distance in degrees
        // currentDistanceInDegrees = normalizeHeading(InertialSensor.heading()) - startingDegrees;
        // currentDistanceInDegrees = currentNormHeading - startingNormHeading;
        Brain.Screen.printAt(10, 160, "RightRPM[0]: %.2f", rightMotorRPM);
        // Brain.Screen.printAt(10, 180, "RightVolt[0]: %.2f", motorVoltageRight[1]);
        Brain.Screen.printAt(10, 180, "LeftEncRPM: %.2f", leftEncoderRPM);
        Brain.Screen.printAt(10, 200, "RightEncRPM: %.2f", rightEncoderRPM);
        Brain.Screen.printAt(10, 220, "AvgEncRPM: %.2f", (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2);

        // avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2])/6;

        // Launch Phase
        // if (std::fabs(currentDistanceInDegrees) < (std::fabs(targetDistanceInDegrees) - breakDistanceInDegrees) && !accelCompleted && !turnCompleted) {
        if ((std::fabs(headingError) > fabs(breakDistanceInDegrees)) && !accelCompleted && !decel)
        {

            // Call traction control class and get adjusted motor voltage
        double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);
        double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, averageEncoderRPM, TURN_ACCEL_FACTOR_LAUNCH);

        // Find the minimum MAGNITUDE (most conservative)
        double syncedMotorVoltage = std::min(fabs(leftTractionVoltage), fabs(rightTractionVoltage));

        // Apply to all 3 motors on both sides with original signs preserved
        for (int i = 0; i < 3; i++)
        {
            motorVoltageLeft[i] = std::copysign(syncedMotorVoltage, motorVoltageLeft[i]);
            motorVoltageRight[i] = std::copysign(syncedMotorVoltage, motorVoltageRight[i]);
        }

            averageMotorVoltage = (fabs(motorVoltageLeft[0]) + fabs(motorVoltageRight[0]) + fabs(motorVoltageLeft[1]) + fabs(motorVoltageRight[1]) + fabs(motorVoltageLeft[2]) + fabs(motorVoltageRight[2])) / numberDriveMotor;
           
            // Update rolling average
            voltageRollingAverage = rollingAverage(averageMotorVoltage, voltageRollingAverage, 5);

            if (fabs(voltageRollingAverage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE)){
            accelCompleted = true;
            Brain.Screen.printAt(10, 20, "Launch Phase");}
            
        }

        // Cruise Phase
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
        
        }
        else if ((std::abs(headingError) <= fabs(breakDistanceInDegrees)) && decelCompleted == false)
     {
        if (!decel)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = 0;
                motorVoltageRight[i] = 0;
            }
        }
        Brain.Screen.printAt(10, 20, "Decel Phase");
        decel = true;

        vex::brakeType leftBrakeMode = ABSControllerLeft.ABSSpeedReduction(leftMotorRPM, leftEncoderRPM);
        vex::brakeType rightBrakeMode = ABSControllerRight.ABSSpeedReduction(rightMotorRPM, rightEncoderRPM);

        brakeType syncedBrakeMode;

        if (leftBrakeMode == brakeType::coast || rightBrakeMode == brakeType::coast) {
            syncedBrakeMode = brakeType::coast;
        } else {
            syncedBrakeMode = brakeType::brake;
        }

        for (int i = 0; i < 3; i++)
        {
            motorVoltageLeft[i] = 0;
            motorVoltageRight[i] = 0;
            leftMotor[i].setBrake(syncedBrakeMode);
            rightMotor[i].setBrake(syncedBrakeMode);
        }
        leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
        rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

        // Detect if robot slowed down to target minimum speed
        if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
            fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
        {
            decelCompleted = true;
        }
        }

        //Final Approach Phase
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
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
}

// Traction Control Constructor implementation
tractionControl::tractionControl(double minSpeedVoltage, double maxSpeedVoltage, double slipThreshold)
    : minSpeedVoltage(minSpeedVoltage), maxSpeedVoltage(maxSpeedVoltage), slipThreshold(slipThreshold) {}

// Method to determine and adjust motor voltage based on wheel slip
double tractionControl::tractionControlSpeed(double motorVoltage, double wheelSpeed, double robotSpeed, double accelFactor)
{

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
       leftMotor[i].setBrake(coast);
       rightMotor[i].setBrake(coast);  
       leftMotor[i].stop();
       rightMotor[i].stop();
    }
}

void straightOdometry(double targetDistance,
                      double breakDistance,
                      double targetHeading,
                      double minSpeed,
                      double kp_heading,
                      double ki_heading,
                      double kd_heading,
                      double accelHeadingScaling,
                      double decelHeadingScaling,
                      double approachHeadingScaling,
                      double maxSpeed)
{

    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    const double LAUNCH_VOLTAGE = 6;           // Starting voltage, must be higher than 0 
    const double ACCEL_FACTOR_LAUNCH = 1.25;     // Acceleration rate MUST be > 1.0
    // slipThreshold: 0-1 range (0 = no slip allowed, 1 = full slip allowed, .15-.25 = optimal slip)
    const double SLIP_THRESHOLD_TRACTION = 0.3; // Slip threshold 1 is always power, 0 is no power
    const double SLIP_THRESHOLD_ABS = 0.25;      // ABS threshold 1 is hard brake, 0 is no brake
    // ========================================

    // Add timer for acceleration phase
    vex::timer accelTimer;

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
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);



    // RPM Parameters
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM; // Adjusted for frictional losses
    double maxEncoderRPM = 0;
    // Motor Arrays
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftMotorRPM = 0;
    double rightMotorRPM = 0;

    // PID and Heading Control

    // double normTargetHeading = normHeading(targetHeading);
    double avgMotorVoltage = 0; // Used for phase transition checking
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;

  // TEMPORARY - Voltage tracking for debugging traction control
    double maxLeftVoltageReached = 0;
    double maxRightVoltageReached = 0;
    double maxAvgLeftVoltage = 0;
    double maxAvgRightVoltage = 0;
    double maxLeftMotor[3] = {0, 0, 0};   // ← ADD THIS
    double maxRightMotor[3] = {0, 0, 0};  // ← ADD THIS

    // Declaration for slip threshold
    ABSController ABSControllerLeft(SLIP_THRESHOLD_ABS);
    ABSController ABSControllerRight(SLIP_THRESHOLD_ABS);

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

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
       // Use continuous rotation instead of heading()
        double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        // Get encoder speeds FIRST
        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        // Get motor RPMs
  // Get middle motor RPM only (index 1 - traction wheel)
        double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        { // will keep going until acceleration is complete and not in decel
          //  Brain.Screen.printAt(10, 20, "Launch Phase");

           

                // Print initial values
                // Brain.Screen.printAt(10, 80, "Init L: %.2f, R: %.2f", motorVoltageLeft[i], motorVoltageRight[i]);

                // Call traction cotrol class and get adjusted motor voltage
                double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
                double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);

                // Synchronized control - use the lower voltage (more conservative) for BOTH sides
                double syncedMotorVoltage = std::min(leftTractionVoltage, rightTractionVoltage);

                // Apply to all 3 motors on both sides with PID correction
            // Apply to all 3 motors on both sides with PID correction - CORRECTED SIGNS
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
            }

                maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));
            

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
           // Brain.Screen.printAt(10, 20, "Cruise Phase"); // Else condition - specify actions here if the if condition is not met

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
           /*
            if (!decel)
            {
                for (int i = 0; i < 3; i++)
                {
                    motorVoltageLeft[i] = minSpeedVoltage;
                    motorVoltageRight[i] = minSpeedVoltage;
                }
            }
            */    
            decel = true;

            vex::brakeType leftBrakeMode = ABSControllerLeft.ABSSpeedReduction(leftMotorRPM, leftEncoderRPM);
            vex::brakeType rightBrakeMode = ABSControllerRight.ABSSpeedReduction(rightMotorRPM, rightEncoderRPM);

            // Apply PID correction - FLIP THE SIGNS from acceleration
            for (int i = 0; i < 3; i++)
            {
                // If drifting right, apply power to LEFT wheel (opposite of acceleration)
                motorVoltageLeft[i] = std::max(0.0, (headingCorrection * decelHeadingScaling));
                motorVoltageRight[i] = std::max(0.0,-(headingCorrection * decelHeadingScaling));
                
                // Handle left side based on lockup
                if (leftBrakeMode == brakeType::coast)
                {
                    leftMotor[i].setBrake(coast);
                    motorVoltageLeft[i] = 0;
                }
                else if (motorVoltageLeft[i] == 0)
                {
                    leftMotor[i].setBrake(brake);
                }
                
                // Handle right side based on lockup
                if (rightBrakeMode == brakeType::coast)
                {
                    rightMotor[i].setBrake(coast);
                    motorVoltageRight[i] = 0;
                }
                else if (motorVoltageRight[i] == 0)
                {
                    rightMotor[i].setBrake(brake);
                }
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) ||
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }
        
        //Final Approach Phase
        else if (decelCompleted == true)
        {
            // break;
        //    Brain.Screen.printAt(10, 20, "Approach Phase");
        //    Brain.Screen.printAt(10, 40, "Decel Compl: %d", decelCompleted);

            for (int i = 0; i < 3; i++)
            {
                // Example action: Set motor voltage to target voltage directly
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
                // PIDVoltageCapCorrection(motorVoltageLeft[i], motorVoltageRight[i], absoluteMaxVoltage);
            }
        }

// Track maximum voltages - both individual motors AND averages
for (int i = 0; i < 3; i++)
{
    // Track highest voltage for ANY motor on each side
    maxLeftVoltageReached = std::max(maxLeftVoltageReached, std::fabs(motorVoltageLeft[i]));
    maxRightVoltageReached = std::max(maxRightVoltageReached, std::fabs(motorVoltageRight[i]));
    
    maxLeftMotor[i] = std::max(maxLeftMotor[i], std::fabs(motorVoltageLeft[i]));
    maxRightMotor[i] = std::max(maxRightMotor[i], std::fabs(motorVoltageRight[i]));
}

// Calculate and track average voltage across all motors
double currentAvgLeftVoltage = (std::fabs(motorVoltageLeft[0]) + 
                                std::fabs(motorVoltageLeft[1]) + 
                                std::fabs(motorVoltageLeft[2])) / 3.0;

double currentAvgRightVoltage = (std::fabs(motorVoltageRight[0]) + 
                                 std::fabs(motorVoltageRight[1]) + 
                                 std::fabs(motorVoltageRight[2])) / 3.0;

static double maxAvgLeftVoltage = 0;
static double maxAvgRightVoltage = 0;
maxAvgLeftVoltage = std::max(maxAvgLeftVoltage, currentAvgLeftVoltage);
maxAvgRightVoltage = std::max(maxAvgRightVoltage, currentAvgRightVoltage);

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
        
Brain.Screen.clearScreen();
Brain.Screen.setCursor(1, 1);
Brain.Screen.print("=== Movement Complete ===");

Brain.Screen.setCursor(2, 1);
Brain.Screen.print("LEFT: %.1f %.1f %.1f", 
                   maxLeftMotor[0], maxLeftMotor[1], maxLeftMotor[2]);

Brain.Screen.setCursor(3, 1);
Brain.Screen.print("RIGHT: %.1f %.1f %.1f", 
                   maxRightMotor[0], maxRightMotor[1], maxRightMotor[2]);

Brain.Screen.setCursor(4, 1);
Brain.Screen.print("MaxL: %.2f MaxR: %.2f", 
                   maxLeftVoltageReached, maxRightVoltageReached);

Brain.Screen.setCursor(5, 1);
Brain.Screen.print("AvgL: %.2f AvgR: %.2f", 
                   maxAvgLeftVoltage, maxAvgRightVoltage);

Brain.Screen.setCursor(6, 1);
Brain.Screen.print("Target: %.2fV", fabs(maxSpeedVoltage));

Brain.Screen.setCursor(7, 1);
Brain.Screen.print("Distance: %.1f / %.1f", 
                   fabs(currentDistance), fabs(targetDistance));

        vex::task::sleep(10);
    }

    // Stop all motors at end of routine after approach
 // Stop all motors at end of routine after approach
    for (int i = 0; i < 3; i++)
    {
       leftMotor[i].setBrake(brake);
       rightMotor[i].setBrake(brake);  
       leftMotor[i].stop();
       rightMotor[i].stop();
    }

    // Display detailed movement summary with all 6 motors
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("=== Movement Complete ===");

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("LEFT: %.1f %.1f %.1f", 
                       maxLeftMotor[0], maxLeftMotor[1], maxLeftMotor[2]);

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("RIGHT: %.1f %.1f %.1f", 
                       maxRightMotor[0], maxRightMotor[1], maxRightMotor[2]);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("MaxL: %.2f MaxR: %.2f", 
                       maxLeftVoltageReached, maxRightVoltageReached);

    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("AvgL: %.2f AvgR: %.2f", 
                       maxAvgLeftVoltage, maxAvgRightVoltage);

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Target: %.2fV", fabs(maxSpeedVoltage));

    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("Distance: %.1f / %.1f", 
                       fabs(currentDistance), fabs(targetDistance));
                       
    //wait(10000, msec);
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
                     double targetHeading,
                     double minSpeed,
                     double kp_heading, 
                     double ki_heading,
                     double kd_heading, 
                     double accelHeadingScaling,
                     double decelHeadingScaling, 
                     double approachHeadingScaling,
                     double maxSpeed) {
    // No conversion needed - use rotation directly
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
}

//Backeward wrapper to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void backwardMP(double targetDistance,
                      double breakDistance, 
                      double targetHeading,
                      double minSpeed,
                      double kp_heading, 
                      double ki_heading,
                      double kd_heading, 
                      double accelHeadingScaling,
                      double decelHeadingScaling, 
                      double approachHeadingScaling,
                      double maxSpeed) {
    // Force negative distance for backward movement
    targetDistance = -std::fabs(targetDistance);
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
}

//Turn left to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void leftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {

    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // CCW = positive turn, so add the turn amount
    double targetRotation = currentHeading - turnAmount;
    
    turnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
   
}

//Turn right to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void rightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // CW = negative turn, so subtract the turn amount
    double targetRotation = currentHeading + turnAmount;
    
    turnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);

}


void pivotTurnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed)
{
    // Reset completion flags
    
    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    double headingError = targetHeading - currentHeading;
    double currentDistanceInDegrees = headingError;

    //int turnDirection = (headingError > 0) ? -1 : 1;

    Brain.Screen.printAt(10, 40, "Target Rotation: %.2f", targetHeading);
    Brain.Screen.printAt(10, 60, "Curr Rotation: %.2f", currentHeading);

    // Convert % Speed input to voltage with max voltage of 12
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * 12, headingError);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * 12, headingError);
    double launchVoltage = std::copysign(5, headingError);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(launchVoltage)), headingError);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    
    // Maximum slip threshold for spot turns before reducing power
    // Range: 0-1, where:
    // 0 = No slip allowed (most conservative)
    // 1 = Full slip allowed (most aggressive)
    // 0.25 = 25% slip tolerance for balanced control
    // may need to go above 25% given built in difference between encoder and wheel spin speed
    const double SLIP_THRESHOLD_TRACTION = 0.35; //somwewhere between 40 to 60 seems good, at least for 180 turns. 45 seems pretty good.
    const double SLIP_THRESHOLD_ABS = 0.35;

    double accelFactorLaunch = 1.4;
    
     // 0.857143
    // double ABSLockThresholdSpotTurn = 0;
    // double totalMotorRadiansPerSecond = 0.0;
    // double minRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (minSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert minspeed to percentage first
    // double maxRadiansPerSecond = (2 * fabs(absoluteMaxRPM * (maxSpeed * .01) * (wheelCircumferenceCM / 60.0))) / trackWidth; //convert maxspeed to percentage first
    // double turnDirection = 1;
    
    
    // double targetDriveVoltageLeft = 0;
    // double targetDriveVoltageRight = 0;
    // double motorRadiansPerSecondLeft[3];
    // double motorRadiansPerSecondRight[3];
    
    
    
    
    double leftMotorRPM[3] = {0, 0, 0};
    double rightMotorRPM[3] = {0, 0, 0};
    
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
        currentHeading = InertialSensor.rotation(degrees) + headingOffset;
        headingError = targetHeading - currentHeading;
        currentDistanceInDegrees = headingError;

        Brain.Screen.printAt(10, 40, "Target Rotation: %.2f", targetHeading);
        Brain.Screen.printAt(10, 60, "Curr Rotation: %.2f", currentHeading);
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

        // Power Drive Motors - Modified for pivot turn

        // turnDirection = std::copysign(turnDirection, normTargetHeading);
        if (!decel == true || decelCompleted == true)
{
    // Determine which side to pivot based on the sign of the voltage
    // which was already set based on the normalized heading error
    if (motorVoltageLeft[0] > 0) { // Positive voltage - pivot around right side
        // Right side stationary, left side moves
        leftMotor[0].spin(forward, motorVoltageLeft[0], voltageUnits::volt);
        rightMotor[0].stop(brake);
        leftMotor[1].spin(forward, motorVoltageLeft[1], voltageUnits::volt);
        rightMotor[1].stop(hold);
        leftMotor[2].spin(forward, motorVoltageLeft[2], voltageUnits::volt);
        rightMotor[2].stop(brake);
    } else { // Negative voltage - pivot around left side
        // Left side stationary, right side moves
        leftMotor[0].stop(brake); 
        rightMotor[0].spin(forward, fabs(motorVoltageRight[0]), voltageUnits::volt);
        leftMotor[1].stop(hold);
        rightMotor[1].spin(forward, fabs(motorVoltageRight[1]), voltageUnits::volt);
        leftMotor[2].stop(brake);
        rightMotor[2].spin(forward, fabs(motorVoltageRight[2]), voltageUnits::volt);
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

//Turn left to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // CCW = positive turn, so add the turn amount
    double targetRotation = currentHeading + turnAmount;
    
    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

//Turn right to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // CW = negative turn, so subtract the turn amount
    double targetRotation = currentHeading - turnAmount;
    
    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

//=============================================================================
// ABSOLUTE HEADING WRAPPER FUNCTIONS - FINAL VERSION
// Add these functions to your navigation.cpp file
//=============================================================================

//Forward wrapper - uses absolute heading with forward as 0°
//Forward wrapper - uses absolute heading with forward as 0°
void driveForward(double targetDistance,
             double breakDistance, 
             double targetHeading,
             double minSpeed,
             double kp_heading, 
             double ki_heading,
             double kd_heading, 
             double accelHeadingScaling,
             double decelHeadingScaling, 
             double approachHeadingScaling,
             double maxSpeed) {
    // MATCH the coordinate system used by turnRight/turnLeft
    double internalHeading = -targetHeading;  // ✅ WITH FLIPPING
    
    straightOdometry(targetDistance, breakDistance, internalHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
}

void driveBackward(double targetDistance,
              double breakDistance, 
              double targetHeading,
              double minSpeed,
              double kp_heading, 
              double ki_heading,
              double kd_heading, 
              double accelHeadingScaling,
              double decelHeadingScaling, 
              double approachHeadingScaling,
              double maxSpeed) {
    targetDistance = -std::fabs(targetDistance);
    
    // SAME coordinate system as forward and turns
    double internalHeading = -targetHeading;  // ✅ WITH FLIPPING
    
    straightOdometry(targetDistance, breakDistance, internalHeading, minSpeed,
                    kp_heading, ki_heading, kd_heading,
                    accelHeadingScaling, decelHeadingScaling,
                    approachHeadingScaling, maxSpeed);
}

//Turn right to absolute heading - FORCES CLOCKWISE DIRECTION
void turnRight(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current heading in robot coordinate system
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // Start with the target in robot coordinate system
    double targetHeading = -absoluteTargetHeading;
    
    // FORCE clockwise by making target higher than current (positive error)
    // Keep adding 360° until target > current (this forces CW motion)
    while (targetHeading <= currentHeading) {
        targetHeading += 360.0;
    }
    
    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed);
}

//Turn left to absolute heading - FORCES COUNTER-CLOCKWISE DIRECTION
void turnLeft(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed) {
    // Get current heading in robot coordinate system
    double currentHeading = InertialSensor.rotation(degrees) + headingOffset;
    
    // Start with the target in robot coordinate system
    double targetHeading = -absoluteTargetHeading;
    
    // FORCE counter-clockwise by making target lower than current (negative error)
    // Keep subtracting 360° until target < current (this forces CCW motion)
    while (targetHeading >= currentHeading) {
        targetHeading -= 360.0;
    }
    
    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed);
}

void intake(bool state, double speedPct){
    if (state == true){
       intakeMotor1.spin(reverse, (speedPct/8.34), voltageUnits::volt);
       intakeMotor2.spin(reverse, (speedPct/8.34), voltageUnits::volt);
    }
    else {
        intakeMotor1.stop();
        intakeMotor2.stop();
    }
}

//intake only
void intake2(double time, bool pistonState) //time in milliseconds, true for pistons, false for no pistons
{
    vex::timer intakeTime;
    intakeTime.reset();

    if (pistonState == true){
        frontHoodPneumatics.set(true); //close front hood and open back hood
        backHoodPneumatics.set(false);
    }
    else {
        frontHoodPneumatics.set(false); //no pistons
        backHoodPneumatics.set(false);
    }

    while (intakeTime.time(timeUnits::msec) < time){
        intakeMotor1.spin(reverse, 12.0, voltageUnits::volt);
        intakeMotor2.spin(reverse, 12.0, voltageUnits::volt);
        vex::task::sleep(10);
    }
    backHoodPneumatics.set(false); //close back hood after intake
    intakeMotor1.stop();
    intakeMotor2.stop();
}

//asynchronous intake
static std::atomic<bool> g_intakeTaskRunning(false);
static double g_intakeTimeMs = 0;
static double g_intakePct = 100;
static bool g_intakePistonState = false;
static bool g_matchLoadState = false;
static vex::task g_intakeTaskHandle; //will hold the spawned task

int intakeTaskEntry(void*) {
    g_intakeTaskRunning.store(true);

    //set pistons once at start
    if (g_matchLoadState) {
        matchLoadPneumatics.set(true);
    } else {
        matchLoadPneumatics.set(false);
    }

    if (g_intakePistonState) {
        frontHoodPneumatics.set(true);
        backHoodPneumatics.set(false);
    } else {
        frontHoodPneumatics.set(false);
        backHoodPneumatics.set(false);
    }

    vex::timer t;
    t.reset();
    double intakeVoltage = g_intakePct / 8.34;
    while (g_intakeTaskRunning.load() && t.time(timeUnits::msec) < g_intakeTimeMs) {
        intakeMotor1.spin(reverse, intakeVoltage, voltageUnits::volt);
        intakeMotor2.spin(reverse, intakeVoltage, voltageUnits::volt);
        vex::task::sleep(10);
    }

    //cleanup
    intakeMotor1.stop();
    intakeMotor2.stop();
    frontHoodPneumatics.set(false);
    backHoodPneumatics.set(false);
    matchLoadPneumatics.set(false);
    g_intakeTaskRunning.store(false);

    return 0;
}

//start intake asynchronously
void intakeStart(double timeMs, double intakePct, bool pistonState, bool matchLoad) {
    // if already running, stop previous then start new
    if (g_intakeTaskRunning.load()) {   
        g_intakeTaskRunning.store(false);
        vex::task::sleep(20);
    }
    g_intakeTimeMs = timeMs;
    g_intakePistonState = pistonState;
    g_intakePct = intakePct;
    g_matchLoadState = matchLoad;
    g_intakeTaskHandle = vex::task(intakeTaskEntry, nullptr);
}

//stop the async intake task early
void intakeStop() {
    g_intakeTaskRunning.store(false);
    vex::task::sleep(20); // allow task to clean up
}

void score(double time, double power) //skibidi score
{
    vex::timer scoringTime;
    scoringTime.reset();

    frontHoodPneumatics.set(false); //open front hood and close back hood
    backHoodPneumatics.set(true);
    ptoPneumatics.set(true); //engage pto for scoring

    double voltagePower = (power / 8.34); //convert power percentage to voltage

    //(scoringTime.time(timeUnits::msec) < time) voltagePower -= time * 0.006;
    while (scoringTime.time(timeUnits::msec) < time)
    {  
        intakeMotor1.spin(reverse, voltagePower, voltageUnits::volt);
        intakeMotor2.spin(reverse, voltagePower, voltageUnits::volt);
        vex::task::sleep(10);
    }
    frontHoodPneumatics.set(false); //close back hood after scoring
    intakeMotor1.stop();
    intakeMotor2.stop();
    ptoPneumatics.set(false); //disengage pto after scoring
}

void stopScore(){
    intakeMotor1.stop();
    intakeMotor2.stop();
}

void outtake(double time) //skibidi outtake
{
    vex::timer outtakeTime;
    outtakeTime.reset();

        ptoPneumatics.set(true); //engage pto for outtake
    frontHoodPneumatics.set(false); //close front hood for outtake
    backHoodPneumatics.set(false); //close back hood for outtake

    while (outtakeTime.time(timeUnits::msec) < time)
    {
        intakeMotor1.spin(forward, 12.0, voltageUnits::volt);
        intakeMotor2.spin(forward, 12.0, voltageUnits::volt);
        vex::task::sleep(10);
    }
    ptoPneumatics.set(false); //disengage pto after outtake
    intakeMotor1.stop();
    intakeMotor2.stop();
}

void stopOuttake(){
    intakeMotor1.stop();
    intakeMotor2.stop();
}