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

const int VISION_CENTER_X = 160;      // Center of 320px wide sensor frame
const int MIN_OBJECT_WIDTH = 25;      // Minimum width in pixels for valid block detection

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

void smartMove(double distanceCM, double maxSpeed, vex::directionType dir, double wallStalledTimeMs)
{
    // Stall detection config
    const double WALL_STOP_THRESHOLD_RPM = 5.0;
    vex::timer wallStallTimer;
    bool wallDetected = false;
    bool wallDetectEnabled = (wallStalledTimeMs > 0);
    bool isCurrentlyStalled = false;

    // Reset encoders
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    // Set brake mode
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(brakeType::coast);
        rightMotor[i].setBrake(brakeType::coast);
    }

    // Convert speed to voltage
    double voltage = maxSpeed * 0.01 * absoluteMaxVoltage;
    if (dir == vex::reverse)
    {
        voltage = -voltage;
    }

    // Main loop
    double currentDistance = 0;
    while (fabs(currentDistance) < fabs(distanceCM) && !wallDetected)
    {
        // Update distance from encoders
        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;

        // Get encoder speeds for stall detection
        double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm));
        double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm));

        // ========================================
        // STALL DETECTION LOGIC
        // ========================================
        if (wallDetectEnabled)
        {
            double avgEncoderSpeed = (leftEncoderRPM + rightEncoderRPM) / 2.0;
            
            if (avgEncoderSpeed < WALL_STOP_THRESHOLD_RPM)
            {
                if (!isCurrentlyStalled)
                {
                    wallStallTimer.reset();
                    isCurrentlyStalled = true;
                }
                else if (wallStallTimer.time(msec) >= wallStalledTimeMs)
                {
                    wallDetected = true;
                }
            }
            else
            {
                isCurrentlyStalled = false;
            }
        }
        // ========================================

        // Spin motors
        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, voltage, voltageUnits::volt);
            rightMotor[i].spin(forward, voltage, voltageUnits::volt);
        }

        vex::task::sleep(10);
    }

    // Stop all motors
    //vex::task::sleep(800);
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].stop(brake);
        rightMotor[i].stop(brake);
    }
}

void turnOdometry(double targetHeading, double breakDistanceInDegrees, double minSpeed, double maxSpeed, double exitTolerance)
{
    // Reset completion flags

    bool decelCompleted = false;
    bool accelCompleted = false;
    bool decel = false;

    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;
    int completeRotations = (int)(currentHeading / 360.0);
    double targetRotationHeading = targetHeading + (completeRotations * 360.0);
    double headingError = targetRotationHeading - currentHeading;
    // double currentDistanceInDegrees = headingError;

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
    const double TURN_ACCEL_FACTOR_LAUNCH = 1.5;
    const double SLIP_THRESHOLD_TRACTION = 10; // somwewhere between 40 to 60 seems good, at least for 180 turns. 45 seems pretty good.
    // Adaptive ABS configuration
    const double DECEL_STEP_PERCENT = 20;     // Voltage step as % of 12V
    const double LOCK_THRESHOLD_DECEL = 10;// Lockup sensitivity

    //const double EXIT_ROTATION_RATE = 15.0;  // Exit when rotation slows to this (degrees/sec)

    double averageMotorVoltage = 0;
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};  // Initialize all elements to minimum launch speed
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage}; // Initialize all elements to minimum launch speed

    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;
    double headingRateRollingAvg = 0;  

    // Declaration for slip threshold
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Loop to continuously adjust motor power
  while ((maxSpeedVoltage > 0 && currentHeading <= targetRotationHeading - exitTolerance) ||
               (maxSpeedVoltage < 0 && currentHeading >= targetRotationHeading + exitTolerance))
    {
        currentHeading = InertialSensor.rotation(degrees) - headingOffset;
        headingError = targetRotationHeading - currentHeading;
        // currentDistanceInDegrees = headingError;

        Brain.Screen.printAt(10, 100, "Curr Rotation: %.2f", currentHeading);
        // Brain.Screen.printAt(10, 120, "Curr Dist: %.2f", currentDistanceInDegrees);
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

            if (fabs(voltageRollingAverage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {
                accelCompleted = true;
                Brain.Screen.printAt(10, 20, "Launch Phase");
            }
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
            // Initialize ABS on first entry
            if (!decel)
            {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }

            Brain.Screen.printAt(10, 20, "Decel Phase");
            decel = true;

            // Get motor and encoder speeds (already declared above in main loop)
            double leftMotorRPMDecel = fabs(leftMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPMDecel = fabs(rightMotor[1].velocity(vex::velocityUnits::rpm)) * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPMDecel = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                        (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPMDecel = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                        (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            // Calculate brake voltages independently for each side
            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPMDecel, leftEncoderRPMDecel);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPMDecel, rightEncoderRPMDecel);

            // Get brake modes from ABS
            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // SYNC brake mode: if EITHER side is locking up, BOTH sides coast
            vex::brakeType syncedBrakeMode;
            if (leftBrakeMode == vex::coast || rightBrakeMode == vex::coast) {
                syncedBrakeMode = vex::coast;
            } else {
                syncedBrakeMode = vex::brake;
            }

            // For turns: use HIGHER voltage (less aggressive braking) to maintain rotation momentum
            double syncedDecelVoltage = std::max(fabs(leftDecelVoltage), fabs(rightDecelVoltage));

            // Apply synchronized voltage and brake mode to both sides
            for (int i = 0; i < 3; i++)
            {
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);

                if (syncedBrakeMode == vex::brake && syncedDecelVoltage > 0.0)
                {
                    // Apply graduated braking voltage with original sign preserved
                    motorVoltageLeft[i] = std::copysign(syncedDecelVoltage, motorVoltageLeft[i]);
                    motorVoltageRight[i] = std::copysign(syncedDecelVoltage, motorVoltageRight[i]);
                }
                else
                {
                    // Coasting to release lockup - zero voltage
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPMDecel, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPMDecel, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }
        // Final Approach Phase
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

    // Power Drive Motors - apply voltages in ALL phases (including decel)
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
        rightMotor[i].spin(forward, -motorVoltageRight[i], voltageUnits::volt);
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
/*
    vex::task::sleep(20);
        // Stop all motors at end of routine after approach
    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(hold);
        rightMotor[i].setBrake(hold);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
    */
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

// Adaptive ABS Constructor - calculates voltage step from percentage of 12V max
// Adaptive ABS Constructor
adaptiveABS::adaptiveABS(double decelStepPercent, double lockThreshold)
    : lockThreshold(lockThreshold), lastAttemptedVoltage(0.0), wasLockedLastCycle(false), 
      currentBrakeMode(vex::brake)
{
    decelStepVoltage = absoluteMaxVoltage * (decelStepPercent / 100.0);
}

// Initialize ABS at start of decel phase
void adaptiveABS::initialize(double startingVoltage)
{
    lastAttemptedVoltage = startingVoltage;
    wasLockedLastCycle = false;
    currentBrakeMode = vex::brake;
}

// Adaptive ABS control - adjusts brake voltage based on wheel lockup
double adaptiveABS::decelControlSpeed(double wheelSpeed, double robotSpeed)
{
    // Calculate lockup ratio
    double lockupRatio = calculateLockupRatio(wheelSpeed, robotSpeed);
    
    double outputVoltage;
    
    if (lockupRatio > lockThreshold)
    {
        // LOCKED: Return 0V with coast mode
        outputVoltage = 0.0;
        currentBrakeMode = vex::coast;
        wasLockedLastCycle = true;
    }
    else if (wasLockedLastCycle)
    {
        // PREVIOUSLY LOCKED: Retry same voltage with brake mode
        outputVoltage = lastAttemptedVoltage;
        currentBrakeMode = vex::brake;
        wasLockedLastCycle = false;
    }
    else
    {
        // NORMAL: Reduce voltage with brake mode
        lastAttemptedVoltage = std::copysign(
            std::max(0.0, std::fabs(lastAttemptedVoltage) - decelStepVoltage),
            lastAttemptedVoltage
        );
        outputVoltage = lastAttemptedVoltage;
        currentBrakeMode = vex::brake;
    }
    
    return outputVoltage;
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
    // Adaptive Launch Control Configuration
    const double LAUNCH_VOLTAGE = 6;         // Starting voltage, must be higher than 0
    const double ACCEL_FACTOR_LAUNCH = 1.25; // Acceleration rate MUST be > 1.0
    // slipThreshold: 0-1 range (0 = no slip allowed, 1 = full slip allowed, .15-.25 = optimal slip)
    const double SLIP_THRESHOLD_TRACTION = 0.3; // Slip threshold 1 is always power, 0 is no power
    // Adaptive ABS configuration
    const double DECEL_STEP_PERCENT = 20;    // Voltage step as % of 12V (range: 1-10)
    const double LOCK_THRESHOLD_DECEL = 0.25; // Lockup sensitivity (range: 0.15-0.40)
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
    // double headingDirection = (targetDistance > 0) ? 1.0 : -1.0;

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
    int consecutiveAtTarget = 0;
    const int REQUIRED_CONSECUTIVE = 3;

    // TEMPORARY - Voltage tracking for debugging traction control
    double maxLeftVoltageReached = 0;
    double maxRightVoltageReached = 0;
    double maxAvgLeftVoltage = 0;
    double maxAvgRightVoltage = 0;
    double maxLeftMotor[3] = {0, 0, 0};  // ← ADD THIS
    double maxRightMotor[3] = {0, 0, 0}; // ← ADD THIS

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Create ABS instances for left and right sides
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 6)
    {

        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        // Display current encoder distance
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Current: %.2f cm", std::fabs(currentDistance));
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Target: %.2f cm", std::fabs(targetDistance));

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
        double currentHeading = InertialSensor.rotation(degrees) - headingOffset;
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
            // Synchronized control - use the lower MAGNITUDE voltage (more conservative) for BOTH sides
            // For negative values, std::min picks the MORE negative (higher magnitude), so use fabs comparison
            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage 
                : rightTractionVoltage;

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

        // Deceleration phase with adaptive ABS
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
// First entry - initialize ABS
            if (!decel)
            {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }
            decel = true;

            // Get motor and encoder speeds
            double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPM = passiveEncoderLeft.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPM = passiveEncoderRight.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            // Calculate brake voltages using adaptive ABS
            // Each side independently determines how much voltage to apply based on lockup detection
            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Get brake modes from ABS
            // Returns coast if wheel is locking up, brake if wheel has traction
            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // SYNC brake mode: if EITHER side is locking up (coasting), BOTH sides coast
            // This keeps the robot straight - prevents one side braking harder than the other
            vex::brakeType syncedBrakeMode;
            if (leftBrakeMode == vex::coast || rightBrakeMode == vex::coast) {
                syncedBrakeMode = vex::coast;
            } else {
                syncedBrakeMode = vex::brake;
            }

            // SYNC voltage: use the MINIMUM magnitude (most conservative) for BOTH sides
            // Mirrors accel phase pattern - whichever side needs less power dictates both
            // This ensures both sides decelerate at the same rate to maintain straight tracking
            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage
                : rightDecelVoltage;

            // Calculate steering correction scaled for decel phase
            double steeringCorrection = headingCorrection * decelHeadingScaling;

            // Apply synced voltage with heading correction (mirrors accel phase pattern)
            // Pattern: syncedBase + correction (left) / syncedBase - correction (right)
            for (int i = 0; i < 3; i++)
            {
                // Only apply voltage if we're actively braking AND have voltage to apply
                if (syncedBrakeMode == vex::brake && std::fabs(syncedDecelVoltage) > 0.0)
                {
                    double correctedLeft = syncedDecelVoltage + steeringCorrection;
                    double correctedRight = syncedDecelVoltage - steeringCorrection;
                    
                    // Clamp toward zero - don't let steering correction reverse motor direction
                    // Forward (positive voltage): use max(0, x) to prevent negative values
                    // Backward (negative voltage): use min(0, x) to prevent positive values
                    if (syncedDecelVoltage > 0) {
                        motorVoltageLeft[i] = std::max(0.0, correctedLeft);
                        motorVoltageRight[i] = std::max(0.0, correctedRight);
                    } else {
                        motorVoltageLeft[i] = std::min(0.0, correctedLeft);
                        motorVoltageRight[i] = std::min(0.0, correctedRight);
                    }
                }
                else
                {
                    // Coasting to release lockup OR fully stopped - zero voltage
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
                
                // Set synced brake mode for both sides to maintain symmetry
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);
            }

            // Update rolling averages for exit detection
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            // Exit when BOTH sides slowed to minimum speed, confirmed multiple times
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                consecutiveAtTarget++;
                if (consecutiveAtTarget >= REQUIRED_CONSECUTIVE)
                {
                    decelCompleted = true;
                }
            }
            else
            {
                consecutiveAtTarget = 0;  // Reset if we pop back above target
            }
        }

        // Final Approach Phase
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
                                        std::fabs(motorVoltageLeft[2])) /
                                       3.0;

        double currentAvgRightVoltage = (std::fabs(motorVoltageRight[0]) +
                                         std::fabs(motorVoltageRight[1]) +
                                         std::fabs(motorVoltageRight[2])) /
                                        3.0;

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
        /*
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
        */
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

    //wait(120, msec); // brief pause to allow motors to settle
/*
       for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(hold);
        rightMotor[i].setBrake(hold);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
*/
 
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

    // wait(10000, msec);
    /*
        currentDistance = (passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees) ) / 2 * (encoderWheelCircumferenceCM / 360.0);
        // Debug print: Stopping motors
        Brain.Screen.clearLine(8);
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Distance Complete");
        Brain.Screen.print("Current Distance: %.2f", currentDistance);
    */
}

void straightOdometryV2(double targetDistance,
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
                      double maxSpeed)
{
    // ========================================
    // CONFIGURATION CONSTANTS
    // ========================================
    const double LAUNCH_VOLTAGE = 6;
    const double ACCEL_FACTOR_LAUNCH = 1.25;
    const double SLIP_THRESHOLD_TRACTION = 0.3;
    const double DECEL_STEP_PERCENT = 20;
    const double LOCK_THRESHOLD_DECEL = 0.25;
    // ========================================

    vex::timer accelTimer;

    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();

    double currentDistance = 0;

    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);

    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxEncoderRPM = 0;

    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftMotorRPM = 0;
    double rightMotorRPM = 0;

    double avgMotorVoltage = 0;
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    int consecutiveAtTarget = 0;
    const int REQUIRED_CONSECUTIVE = 3;

    double maxLeftVoltageReached = 0;
    double maxRightVoltageReached = 0;
    double maxAvgLeftVoltage = 0;
    double maxAvgRightVoltage = 0;
    double maxLeftMotor[3] = {0, 0, 0};
    double maxRightMotor[3] = {0, 0, 0};

    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    while (std::fabs(currentDistance) <= fabs(targetDistance) - distanceTolerance)
    {
        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Current: %.2f cm", std::fabs(currentDistance));
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Target: %.2f cm", std::fabs(targetDistance));

        double currentHeading = InertialSensor.rotation(degrees) - headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);

            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage 
                : rightTractionVoltage;

            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
            }

            maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));

            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {
                accelCompleted = true;
            }
        }
        // Cruise Phase
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = maxSpeedVoltage + (headingCorrection);
                motorVoltageRight[i] = maxSpeedVoltage - (headingCorrection);
            }
        }
        // Deceleration phase with adaptive ABS
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            if (!decel)
            {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }
            decel = true;

            double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPM = passiveEncoderLeft.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPM = passiveEncoderRight.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            vex::brakeType syncedBrakeMode;
            if (leftBrakeMode == vex::coast || rightBrakeMode == vex::coast) {
                syncedBrakeMode = vex::coast;
            } else {
                syncedBrakeMode = vex::brake;
            }

            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage
                : rightDecelVoltage;

            double steeringCorrection = headingCorrection * decelHeadingScaling;

            for (int i = 0; i < 3; i++)
            {
                if (syncedBrakeMode == vex::brake && std::fabs(syncedDecelVoltage) > 0.0)
                {
                    double correctedLeft = syncedDecelVoltage + steeringCorrection;
                    double correctedRight = syncedDecelVoltage - steeringCorrection;
                    
                    if (syncedDecelVoltage > 0) {
                        motorVoltageLeft[i] = std::max(0.0, correctedLeft);
                        motorVoltageRight[i] = std::max(0.0, correctedRight);
                    } else {
                        motorVoltageLeft[i] = std::min(0.0, correctedLeft);
                        motorVoltageRight[i] = std::min(0.0, correctedRight);
                    }
                }
                else
                {
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
                
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                consecutiveAtTarget++;
                if (consecutiveAtTarget >= REQUIRED_CONSECUTIVE)
                {
                    decelCompleted = true;
                }
            }
            else
            {
                consecutiveAtTarget = 0;
            }
        }
        // Final Approach Phase
        else if (decelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
            }
        }

        for (int i = 0; i < 3; i++)
        {
            maxLeftVoltageReached = std::max(maxLeftVoltageReached, std::fabs(motorVoltageLeft[i]));
            maxRightVoltageReached = std::max(maxRightVoltageReached, std::fabs(motorVoltageRight[i]));

            maxLeftMotor[i] = std::max(maxLeftMotor[i], std::fabs(motorVoltageLeft[i]));
            maxRightMotor[i] = std::max(maxRightMotor[i], std::fabs(motorVoltageRight[i]));
        }

        double currentAvgLeftVoltage = (std::fabs(motorVoltageLeft[0]) +
                                        std::fabs(motorVoltageLeft[1]) +
                                        std::fabs(motorVoltageLeft[2])) /
                                       3.0;

        double currentAvgRightVoltage = (std::fabs(motorVoltageRight[0]) +
                                         std::fabs(motorVoltageRight[1]) +
                                         std::fabs(motorVoltageRight[2])) /
                                        3.0;

        static double maxAvgLeftVoltage = 0;
        static double maxAvgRightVoltage = 0;
        maxAvgLeftVoltage = std::max(maxAvgLeftVoltage, currentAvgLeftVoltage);
        maxAvgRightVoltage = std::max(maxAvgRightVoltage, currentAvgRightVoltage);

        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
        }

        vex::task::sleep(10);
    }

    for (int i = 0; i < 3; i++)
    {
        leftMotor[i].setBrake(brake);
        rightMotor[i].setBrake(brake);
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
}

void smartStraight(double targetDistance,
                      double breakDistance,
                      double targetHeading,
                      double minSpeed,
                      double wallStalledTimeMs,
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
    const double LAUNCH_VOLTAGE = 6;
    const double ACCEL_FACTOR_LAUNCH = 1.25;
    const double SLIP_THRESHOLD_TRACTION = 0.3;
    const double DECEL_STEP_PERCENT = 20;
    const double LOCK_THRESHOLD_DECEL = 0.25;
    
    // Wall Detection
    const double WALL_STOP_THRESHOLD_RPM = 5.0;
    // ========================================

    // Add timer for acceleration phase
    vex::timer accelTimer;

    // Wall detection timer and state
    vex::timer wallStallTimer;
    bool wallDetected = false;
    bool wallDetectEnabled = (wallStalledTimeMs > 0);
    bool isCurrentlyStalled = false;

    // Reset the encoders to start counting from zero
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();

    // Initialize PID controllers
    PID headingPID(kp_heading, ki_heading, kd_heading);
    headingPID.pidReset();
    targetHeading = -targetHeading;

    // Motion Parameters
    double currentDistance = 0;
    // double headingDirection = (targetDistance > 0) ? 1.0 : -1.0;

    // Target Speeds & Voltages
    double maxSpeedVoltage = std::copysign(maxSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double minSpeedVoltage = std::copysign(minSpeed * 0.01 * absoluteMaxVoltage, targetDistance);
    double launchVoltage = std::copysign(LAUNCH_VOLTAGE, targetDistance);
    double minLaunchSpeedVoltage = std::copysign(std::min(fabs(maxSpeedVoltage), fabs(LAUNCH_VOLTAGE)), targetDistance);

    // RPM Parameters
    double minDriveMotorRPM = (minSpeed * .01) * absoluteMaxRPM;
    double maxEncoderRPM = 0;
    
    // Motor Arrays
    double motorVoltageLeft[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double motorVoltageRight[3] = {minLaunchSpeedVoltage, minLaunchSpeedVoltage, minLaunchSpeedVoltage};
    double leftMotorRPM = 0;
    double rightMotorRPM = 0;

    // PID and Heading Control

    // double normTargetHeading = normHeading(targetHeading);
    double avgMotorVoltage = 0;
    double leftEncoderRollingAverage = 0;
    double rightEncoderRollingAverage = 0;
    double voltageRollingAverage = 0;

    bool decel = false;
    bool decelCompleted = false;
    bool accelCompleted = false;
    int consecutiveAtTarget = 0;
    const int REQUIRED_CONSECUTIVE = 3;

    // TEMPORARY - Voltage tracking for debugging traction control
    double maxLeftVoltageReached = 0;
    double maxRightVoltageReached = 0;
    double maxAvgLeftVoltage = 0;
    double maxAvgRightVoltage = 0;
    double maxLeftMotor[3] = {0, 0, 0};
    double maxRightMotor[3] = {0, 0, 0};

    // Declare arrays of Slip Control instances for each wheel
    tractionControl tractionControlLeft(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);
    tractionControl tractionControlRight(minLaunchSpeedVoltage, maxSpeedVoltage, SLIP_THRESHOLD_TRACTION);

    // Create ABS instances for left and right sides
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

    while (std::fabs(currentDistance) <= fabs(targetDistance) - 6.9 && !wallDetected)
    {

        currentDistance = ((passiveEncoderLeft.position(degrees) + passiveEncoderRight.position(degrees)) / 2.0 / 360.0) * encoderWheelCircumferenceCM;
        avgMotorVoltage = (motorVoltageLeft[0] + motorVoltageLeft[1] + motorVoltageLeft[2] + motorVoltageRight[0] + motorVoltageRight[1] + motorVoltageRight[2]) / numberDriveMotor;

        // Display current encoder distance
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Current: %.2f cm", std::fabs(currentDistance));
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Target: %.2f cm", std::fabs(targetDistance));

        // Calculate the heading correction using the PID controller
        double currentHeading = InertialSensor.rotation(degrees) - headingOffset;
        double headingCorrection = headingPID.calculate(targetHeading, currentHeading);

        // Get encoder speeds FIRST
        double leftEncoderRPM = passiveEncoderLeft.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double rightEncoderRPM = passiveEncoderRight.velocity(vex::velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
        double avgEncoderRPM = (leftEncoderRPM + rightEncoderRPM) / 2;

        // Get motor RPMs
        // Get middle motor RPM only (index 1 - traction wheel)
        double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
        double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;

        // ========================================
        // WALL DETECTION LOGIC
        // ========================================
        if (wallDetectEnabled)
        {
            double avgEncoderSpeed = (fabs(leftEncoderRPM) + fabs(rightEncoderRPM)) / 2.0;
            
            if (avgEncoderSpeed < WALL_STOP_THRESHOLD_RPM)
            {
                if (!isCurrentlyStalled)
                {
                    // Just started stalling, reset timer
                    wallStallTimer.reset();
                    isCurrentlyStalled = true;
                }
                else if (wallStallTimer.time(msec) >= wallStalledTimeMs)
                {
                    // Stalled for required duration
                    wallDetected = true;
                }
            }
            else
            {
                // Moving again, reset stall state
                isCurrentlyStalled = false;
            }
        }
        // ========================================

        // Launch Phase
        if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && !accelCompleted && !decel)
        {
            double leftTractionVoltage = tractionControlLeft.tractionControlSpeed(motorVoltageLeft[1], leftMotorRPM, leftEncoderRPM, ACCEL_FACTOR_LAUNCH);
            double rightTractionVoltage = tractionControlRight.tractionControlSpeed(motorVoltageRight[1], rightMotorRPM, rightEncoderRPM, ACCEL_FACTOR_LAUNCH);

            double syncedMotorVoltage = (std::fabs(leftTractionVoltage) < std::fabs(rightTractionVoltage)) 
                ? leftTractionVoltage 
                : rightTractionVoltage;

            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = syncedMotorVoltage + (headingCorrection * accelHeadingScaling);
                motorVoltageRight[i] = syncedMotorVoltage - (headingCorrection * accelHeadingScaling);
            }

            maxEncoderRPM = std::max(maxEncoderRPM, fabs(avgEncoderRPM));

            if (fabs(avgMotorVoltage) >= (fabs(maxSpeedVoltage) - VOLTAGE_TOLERANCE))
            {
                accelCompleted = true;
            }
        }
        // Cruise Phase
        else if (fabs(currentDistance) < (fabs(targetDistance) - breakDistance) && accelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = maxSpeedVoltage + (headingCorrection);
                motorVoltageRight[i] = maxSpeedVoltage - (headingCorrection);
            }
        }
        // Deceleration phase with adaptive ABS
        else if (fabs(currentDistance) >= (fabs(targetDistance) - breakDistance) && decelCompleted == false)
        {
            // First entry - initialize ABS
            if (!decel)
            {
                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }
            decel = true;

            // Get motor and encoder speeds
            double leftMotorRPM = leftMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double rightMotorRPM = rightMotor[1].velocity(vex::velocityUnits::rpm) * DRIVE_MOTOR_RPM_ADJ;
            double leftEncoderRPM = passiveEncoderLeft.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);
            double rightEncoderRPM = passiveEncoderRight.velocity(velocityUnits::rpm) * (encoderWheelCircumferenceCM / wheelCircumferenceCM);

            // Calculate brake voltages using adaptive ABS
            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Get brake modes from ABS
            vex::brakeType leftBrakeMode = adaptiveABSLeft.getBrakeMode();
            vex::brakeType rightBrakeMode = adaptiveABSRight.getBrakeMode();

            // SYNC brake mode: if EITHER side is locking up (coasting), BOTH sides coast
            vex::brakeType syncedBrakeMode;
            if (leftBrakeMode == vex::hold || rightBrakeMode == vex::hold) {
                syncedBrakeMode = vex::hold;
            } else {
                syncedBrakeMode = vex::hold;
            }

            // SYNC voltage: use the MINIMUM magnitude (most conservative) for BOTH sides
            double syncedDecelVoltage = (std::fabs(leftDecelVoltage) < std::fabs(rightDecelVoltage))
                ? leftDecelVoltage
                : rightDecelVoltage;

            // Calculate steering correction scaled for decel phase
            double steeringCorrection = headingCorrection * decelHeadingScaling;

            // Apply synced voltage with heading correction
            for (int i = 0; i < 3; i++)
            {
                if (syncedBrakeMode == vex::brake && std::fabs(syncedDecelVoltage) > 0.0)
                {
                    double correctedLeft = syncedDecelVoltage + steeringCorrection;
                    double correctedRight = syncedDecelVoltage - steeringCorrection;
                    
                    if (syncedDecelVoltage > 0) {
                        motorVoltageLeft[i] = std::max(0.0, correctedLeft);
                        motorVoltageRight[i] = std::max(0.0, correctedRight);
                    } else {
                        motorVoltageLeft[i] = std::min(0.0, correctedLeft);
                        motorVoltageRight[i] = std::min(0.0, correctedRight);
                    }
                }
                else
                {
                    motorVoltageLeft[i] = 0.0;
                    motorVoltageRight[i] = 0.0;
                }
                
                leftMotor[i].setBrake(syncedBrakeMode);
                rightMotor[i].setBrake(syncedBrakeMode);
            }

            // Update rolling averages for exit detection
            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 10);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 10);

            // Exit when BOTH sides slowed to minimum speed, confirmed multiple times
            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                consecutiveAtTarget++;
                if (consecutiveAtTarget >= REQUIRED_CONSECUTIVE)
                {
                    decelCompleted = true;
                }
            }
            else
            {
                consecutiveAtTarget = 0;
            }
        }
        // Final Approach Phase
        else if (decelCompleted == true)
        {
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = minSpeedVoltage + (headingCorrection * approachHeadingScaling);
                motorVoltageRight[i] = minSpeedVoltage - (headingCorrection * approachHeadingScaling);
            }
        }

        // Track maximum voltages - both individual motors AND averages
        for (int i = 0; i < 3; i++)
        {
            maxLeftVoltageReached = std::max(maxLeftVoltageReached, std::fabs(motorVoltageLeft[i]));
            maxRightVoltageReached = std::max(maxRightVoltageReached, std::fabs(motorVoltageRight[i]));

            maxLeftMotor[i] = std::max(maxLeftMotor[i], std::fabs(motorVoltageLeft[i]));
            maxRightMotor[i] = std::max(maxRightMotor[i], std::fabs(motorVoltageRight[i]));
        }

        // Calculate and track average voltage across all motors
        double currentAvgLeftVoltage = (std::fabs(motorVoltageLeft[0]) +
                                        std::fabs(motorVoltageLeft[1]) +
                                        std::fabs(motorVoltageLeft[2])) /
                                       3.0;

        double currentAvgRightVoltage = (std::fabs(motorVoltageRight[0]) +
                                         std::fabs(motorVoltageRight[1]) +
                                         std::fabs(motorVoltageRight[2])) /
                                        3.0;

        static double maxAvgLeftVoltage = 0;
        static double maxAvgRightVoltage = 0;
        maxAvgLeftVoltage = std::max(maxAvgLeftVoltage, currentAvgLeftVoltage);
        maxAvgRightVoltage = std::max(maxAvgRightVoltage, currentAvgRightVoltage);

        // Power Drive Motors
        for (int i = 0; i < 3; i++)
        {
            leftMotor[i].spin(forward, motorVoltageLeft[i], voltageUnits::volt);
            rightMotor[i].spin(forward, motorVoltageRight[i], voltageUnits::volt);
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

    // Display detailed movement summary
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (wallDetected) {
        Brain.Screen.print("=== WALL DETECTED ===");
    } else {
        Brain.Screen.print("=== Movement Complete ===");
    }

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
}

// Forwards wrapper to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
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
               double maxSpeed)
{
    // No conversion needed - use rotation directly
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

// Backeward wrapper to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
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
                double maxSpeed)
{
    // Force negative distance for backward movement
    targetDistance = -std::fabs(targetDistance);
    straightOdometry(targetDistance, breakDistance, targetHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

// Turn left to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void leftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{

    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;

    // CCW = positive turn, so add the turn amount
    double targetRotation = currentHeading - turnAmount;

    turnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

// Turn right to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void rightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{
    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;

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

    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;
    double headingError = targetHeading - currentHeading;
    double currentDistanceInDegrees = headingError;

    // int turnDirection = (headingError > 0) ? -1 : 1;

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
    const double SLIP_THRESHOLD_TRACTION = 0.35; // somwewhere between 40 to 60 seems good, at least for 180 turns. 45 seems pretty good.
    double accelFactorLaunch = 1.4;
    // Adaptive ABS configuration for turns
    const double DECEL_STEP_PERCENT = 2.0;
    const double LOCK_THRESHOLD_DECEL = 0.25;

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
    adaptiveABS adaptiveABSLeft(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);
    adaptiveABS adaptiveABSRight(DECEL_STEP_PERCENT, LOCK_THRESHOLD_DECEL);

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
        currentHeading = InertialSensor.rotation(degrees) - headingOffset;
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
            // Initialize brake mode and ABS on first entry
            if (!decel)
            {
                for (int i = 0; i < 3; i++)
                {
                    leftMotor[i].setBrake(brake);
                    rightMotor[i].setBrake(brake);
                }

                adaptiveABSLeft.initialize(motorVoltageLeft[1]);
                adaptiveABSRight.initialize(motorVoltageRight[1]);
            }

            decel = true;
            Brain.Screen.printAt(10, 20, "Decel Phase");

            // Get motor and encoder speeds
            double leftMotorRPM = fabs(leftMotor[1].velocity(vex::velocityUnits::rpm));
            double rightMotorRPM = fabs(rightMotor[1].velocity(vex::velocityUnits::rpm));
            double leftEncoderRPM = fabs(passiveEncoderLeft.velocity(velocityUnits::rpm)) *
                                    (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;
            double rightEncoderRPM = fabs(passiveEncoderRight.velocity(velocityUnits::rpm)) *
                                     (encoderWheelCircumferenceCM / wheelCircumferenceCM) * ENCODER_RADIUS_RATIO;

            // Calculate brake voltages independently for each side
            double leftDecelVoltage = adaptiveABSLeft.decelControlSpeed(leftMotorRPM, leftEncoderRPM);
            double rightDecelVoltage = adaptiveABSRight.decelControlSpeed(rightMotorRPM, rightEncoderRPM);

            // Synchronize - use HIGHER voltage (less aggressive braking) for both sides
            double syncedDecelVoltage = std::max(fabs(leftDecelVoltage), fabs(rightDecelVoltage));

            // Apply synchronized voltage to both sides
            for (int i = 0; i < 3; i++)
            {
                motorVoltageLeft[i] = std::copysign(syncedDecelVoltage, motorVoltageLeft[i]);
                motorVoltageRight[i] = std::copysign(syncedDecelVoltage, motorVoltageRight[i]);
            }

            leftEncoderRollingAverage = rollingAverage(leftEncoderRPM, leftEncoderRollingAverage, 3);
            rightEncoderRollingAverage = rollingAverage(rightEncoderRPM, rightEncoderRollingAverage, 3);

            if (fabs(leftEncoderRollingAverage) <= fabs(minDriveMotorRPM) &&
                fabs(rightEncoderRollingAverage) <= fabs(minDriveMotorRPM))
            {
                decelCompleted = true;
            }
        }
        // Final Approach Phase
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
            if (motorVoltageLeft[0] > 0)
            { // Positive voltage - pivot around right side
                // Right side stationary, left side moves
                leftMotor[0].spin(forward, motorVoltageLeft[0], voltageUnits::volt);
                rightMotor[0].stop(brake);
                leftMotor[1].spin(forward, motorVoltageLeft[1], voltageUnits::volt);
                rightMotor[1].stop(hold);
                leftMotor[2].spin(forward, motorVoltageLeft[2], voltageUnits::volt);
                rightMotor[2].stop(brake);
            }
            else
            { // Negative voltage - pivot around left side
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

// Turn left to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void pivotLeftMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{
    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;

    // CCW = positive turn, so add the turn amount
    double targetRotation = currentHeading + turnAmount;

    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

// Turn right to call TurnOdometry with motion profiling and convert it from Euclidean CCW (Counter clockwise angles) to CW VEX angles
void pivotRightMP(double turnAmount, double breakDistance, double minSpeed, double maxSpeed)
{
    // Get current rotation
    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;

    // CW = negative turn, so subtract the turn amount
    double targetRotation = currentHeading - turnAmount;

    pivotTurnOdometry(targetRotation, breakDistance, minSpeed, maxSpeed);
}

//=============================================================================
// ABSOLUTE HEADING WRAPPER FUNCTIONS - FINAL VERSION
// Add these functions to your navigation.cpp file
//=============================================================================

// Forward wrapper - uses absolute heading with forward as 0°
// Forward wrapper - uses absolute heading with forward as 0°
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
                  double maxSpeed)
{
    // MATCH the coordinate system used by turnRight/turnLeft
    double internalHeading = -targetHeading; // ✅ WITH FLIPPING

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
                   double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);

    // SAME coordinate system as forward and turns
    double internalHeading = -targetHeading; // ✅ WITH FLIPPING

    straightOdometry(targetDistance, breakDistance, internalHeading, minSpeed,
                     kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

// Turn right to absolute heading - FORCES CLOCKWISE DIRECTION
void turnRight(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance)
{
    // Get current heading in robot coordinate system
    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;

    // Start with the target in robot coordinate system
    double targetHeading = -absoluteTargetHeading;

    // FORCE clockwise by making target higher than current (positive error)
    // Keep adding 360° until target > current (this forces CW motion)
    while (targetHeading <= currentHeading)
    {
        targetHeading += 360.0;
    }

    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

// Turn left to absolute heading - FORCES COUNTER-CLOCKWISE DIRECTION
void turnLeft(double absoluteTargetHeading, double breakDistance, double minSpeed, double maxSpeed, double exitTolerance)
{
    // Get current heading in robot coordinate system
    double currentHeading = InertialSensor.rotation(degrees) - headingOffset;

    // Start with the target in robot coordinate system
    double targetHeading = -absoluteTargetHeading;

    // FORCE counter-clockwise by making target lower than current (negative error)
    // Keep subtracting 360° until target < current (this forces CCW motion)
    while (targetHeading >= currentHeading)
    {
        targetHeading -= 360.0;
    }

    turnOdometry(targetHeading, breakDistance, minSpeed, maxSpeed, exitTolerance);
}

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
                  double maxSpeed)
{
    double internalHeading = -targetHeading;

    straightOdometryV2(targetDistance, breakDistance, internalHeading, minSpeed,
                     distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

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
                   double maxSpeed)
{
    targetDistance = -std::fabs(targetDistance);

    double internalHeading = -targetHeading;

    straightOdometryV2(targetDistance, breakDistance, internalHeading, minSpeed,
                     distanceTolerance, kp_heading, ki_heading, kd_heading,
                     accelHeadingScaling, decelHeadingScaling,
                     approachHeadingScaling, maxSpeed);
}

void pidlessForward(double timeMs, double speedPct)
{
    vex::timer forwardTime;
    forwardTime.reset();

    double voltagePower = (speedPct / 8.34);
    while (forwardTime.time(timeUnits::msec) < timeMs)
    {
        LeftMotor1.spin(forward, voltagePower, voltageUnits::volt);
        LeftMotor2.spin(forward, voltagePower, voltageUnits::volt);
        LeftMotor3.spin(forward, voltagePower, voltageUnits::volt);
        RightMotor1.spin(forward, voltagePower, voltageUnits::volt);
        RightMotor2.spin(forward, voltagePower, voltageUnits::volt);
        RightMotor3.spin(forward, voltagePower, voltageUnits::volt);
        vex::task::sleep(10);
    }
    LeftMotor1.stop(coast);
    LeftMotor2.stop(coast);
    LeftMotor3.stop(coast);
    RightMotor1.stop(coast);
    RightMotor2.stop(coast);
    RightMotor3.stop(coast);
}



void visionDrive(
    vex::aivision::colordesc targetSignature,
    int    targetPixelWidth,
    double timeoutDistanceCM,
    double targetHeading,
    double minSpeedPct,
    double maxSpeedPct,
    vex::brakeType brakeMode,
    double kp_head,
    double ki_head,
    double kd_head,
    double kp_dist,
    double ki_dist,
    double kd_dist,
    int    minX,
    int    maxX,
    int    minY,
    int    maxY,
    int    maxObjectsToCheck,
    int    consecutiveRequired
) {
    // Reset encoders and record starting odometry position for safety timeout
    passiveEncoderLeft.resetPosition();
    passiveEncoderRight.resetPosition();
    updateOdometry();
    double startX = globalX;
    double startY = globalY;

    // Initialize PID controllers for heading (lateral correction) and distance (approach speed)
    PID headingPID(kp_head, ki_head, kd_head);
    PID distPID(kp_dist, ki_dist, kd_dist);
    headingPID.pidReset();
    distPID.pidReset();

    // Buffer for 4-frame rolling median filter on pixel width (reduces noise from bad frames)
    const int MEDIAN_WINDOW = 4;
    double widthHistory[MEDIAN_WINDOW] = {0.0};
    int widthHistoryIdx = 0;
    int widthHistoryCount = 0;

    // Store last valid values for fallback when vision drops
    double lastTurnCorrection = 0.0;
    double lastPixelWidth     = 0.0;  // Starts at 0 → large PID error → full speed if never seen
    int consecutiveStableWidth = 0;

    // Maximum voltage allowed for turn correction (leaves headroom for forward motion)
    const double MAX_TURN_VOLTAGE = 4.0;

    Brain.Screen.clearScreen();

    while (true) {
        // Update current position and check safety distance limit
        updateOdometry();
        double deltaX = globalX - startX;
        double deltaY = globalY - startY;
        double distanceTraveled = sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distanceTraveled >= timeoutDistanceCM) {
            Brain.Screen.printAt(10, 100, "Safety distance timeout!");
            break;
        }

        // Capture latest vision snapshot and find the largest valid object in ROI
        AIVision20.takeSnapshot(targetSignature);

        int bestIdx = -1;
        double largestWidth = 0.0;
        int objectsChecked = std::min(maxObjectsToCheck, (int)AIVision20.objectCount);

        for (int i = 0; i < objectsChecked; i++) {
            auto& obj = AIVision20.objects[i];
            if (obj.width < MIN_OBJECT_WIDTH) continue;

            // Check if object's center is inside the defined bounding box
            if (obj.centerX >= minX && obj.centerX <= maxX &&
                obj.centerY >= minY && obj.centerY <= maxY) {
                if (obj.width > largestWidth) {
                    largestWidth = obj.width;
                    bestIdx = i;
                }
            }
        }

        // Default to last known values (persistence when vision drops)
        double currentPixelWidth = lastPixelWidth;
        double turnCorrection    = lastTurnCorrection;

        if (bestIdx >= 0) {
            auto& obj = AIVision20.objects[bestIdx];

            // Calculate lateral error and apply PID to center object horizontally
            double xError = obj.centerX - VISION_CENTER_X;
            turnCorrection = headingPID.calculate(0.0, xError);

            // Apply 4-frame rolling median to smooth pixel width (rejects outliers)
            double rawWidth = obj.width;
            widthHistory[widthHistoryIdx] = rawWidth;
            widthHistoryIdx = (widthHistoryIdx + 1) % MEDIAN_WINDOW;
            if (widthHistoryCount < MEDIAN_WINDOW) widthHistoryCount++;

            double sorted[MEDIAN_WINDOW];
            std::copy(widthHistory, widthHistory + MEDIAN_WINDOW, sorted);
            std::sort(sorted, sorted + widthHistoryCount);

            if (widthHistoryCount < MEDIAN_WINDOW) {
                currentPixelWidth = rawWidth;
            } else {
                currentPixelWidth = (sorted[1] + sorted[2]) / 2.0;  // Average of two middle values
            }

            // Update persistence for next frame fallback
            lastTurnCorrection = turnCorrection;
            lastPixelWidth     = currentPixelWidth;
        } else {
            // No valid object → fall back to IMU-based heading hold
            double currentHeading = InertialSensor.rotation(degrees) - headingOffset;
            double headingError = targetHeading - currentHeading;
            headingError = fmod(headingError + 540.0, 360.0) - 180.0;
            turnCorrection = headingPID.calculate(targetHeading, currentHeading);
        }

        // Check if we've reached the target width stably
        if (currentPixelWidth >= targetPixelWidth) {
            consecutiveStableWidth++;
            if (consecutiveStableWidth >= consecutiveRequired) {
                break;
            }
        } else {
            consecutiveStableWidth = 0;
        }

        // Calculate forward drive voltage using pixel-width error
        double drivePower = distPID.calculate((double)targetPixelWidth, currentPixelWidth);
        double driveVoltage = drivePower * 0.12;

        // Prevent reverse motion
        if (driveVoltage < 0) driveVoltage = 0;

        // Clamp base drive to user-defined min/max speed FIRST (respects maxSpeedPct)
        double maxVolts = maxSpeedPct * 0.12;
        double minVolts = minSpeedPct * 0.12;
        driveVoltage = std::max(minVolts, std::min(maxVolts, driveVoltage));

        // Clamp turn correction voltage
        double turnVoltage = turnCorrection * 0.12;
        turnVoltage = std::max(-MAX_TURN_VOLTAGE, std::min(MAX_TURN_VOLTAGE, turnVoltage));

        // Apply differential drive
        double leftVolts  = driveVoltage + turnVoltage;
        double rightVolts = driveVoltage - turnVoltage;

        // Final clamp to hardware voltage limits (±12 V)
        leftVolts  = std::max(-12.0, std::min(12.0, leftVolts));
        rightVolts = std::max(-12.0, std::min(12.0, rightVolts));

        // Send voltages to motors
        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward, leftVolts, voltageUnits::volt);
            rightMotor[i].spin(forward, rightVolts, voltageUnits::volt);
        }

        vex::task::sleep(20);
    }

    // Stop motors with chosen brake mode
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}



// ───────────────────────────────────────────────
// Minimal vision drive - core PID only, no safety nets
// ───────────────────────────────────────────────
void visionDriveMinimal(
    vex::aivision::colordesc targetSignature,
    int    targetPixelWidth,
    double targetHeading,
    double minSpeedPct,
    double maxSpeedPct,
    vex::brakeType brakeMode,
    double kp_head,
    double ki_head,
    double kd_head,
    double kp_dist,
    double ki_dist,
    double kd_dist
) {
    PID headingPID(kp_head, ki_head, kd_head);
    PID distPID(kp_dist, ki_dist, kd_dist);
    headingPID.pidReset();
    distPID.pidReset();

    Brain.Screen.clearScreen();

    while (true) {
        AIVision20.takeSnapshot(targetSignature);

        if (AIVision20.objectCount == 0) {
            Brain.Screen.printAt(10, 20, "No object detected - stopping");
            break;
        }

        auto& obj = AIVision20.objects[0];

        double xError = obj.centerX - VISION_CENTER_X;
        double turnCorrection = headingPID.calculate(0.0, xError);

        double currentPixelWidth = obj.width;
        double drivePower = distPID.calculate((double)targetPixelWidth, currentPixelWidth);

        double driveVoltage = drivePower * 0.12;
        if (driveVoltage < 0) driveVoltage = 0;

        double maxVolts = maxSpeedPct * 0.12;
        double minVolts = minSpeedPct * 0.12;
        driveVoltage = std::max(minVolts, std::min(maxVolts, driveVoltage));

        double turnVoltage = turnCorrection * 0.12;

        // ─── FLIPPED turn direction ───
        double leftVolts  = driveVoltage - turnVoltage;
        double rightVolts = driveVoltage + turnVoltage;

        leftVolts  = std::max(-12.0, std::min(12.0, leftVolts));
        rightVolts = std::max(-12.0, std::min(12.0, rightVolts));

        for (int i = 0; i < 3; i++) {
            leftMotor[i].spin(forward,  leftVolts,  voltageUnits::volt);
            rightMotor[i].spin(forward, rightVolts, voltageUnits::volt);
        }

        // Debug prints (comment out when no longer needed)
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 20,  "Obj CenterX: %.0f", obj.centerX);
        Brain.Screen.printAt(10, 40,  "xError: %.2f", xError);
        Brain.Screen.printAt(10, 60,  "turnCorrection: %.2f", turnCorrection);
        Brain.Screen.printAt(10, 80,  "turnVoltage: %.2f", turnVoltage);
        Brain.Screen.printAt(10, 100, "Obj Width: %.0f", currentPixelWidth);
        Brain.Screen.printAt(10, 120, "drivePower: %.2f", drivePower);
        Brain.Screen.printAt(10, 140, "driveVoltage: %.2f", driveVoltage);
        Brain.Screen.printAt(10, 160, "Left Volts: %.2f", leftVolts);
        Brain.Screen.printAt(10, 180, "Right Volts: %.2f", rightVolts);

        if (currentPixelWidth >= targetPixelWidth) {
            Brain.Screen.printAt(10, 200, "Target width reached - stopping");
            break;
        }

        vex::task::sleep(20);
    }

    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop(brakeMode);
        rightMotor[i].stop(brakeMode);
    }
}