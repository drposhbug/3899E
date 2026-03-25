#include "vision_follow.h"
#include "robot_config.h"
#include "vex.h"

using namespace vex;
using namespace VisionFollow;

// PID controller variables
double turnError = 0, lastTurnError = 0, turnIntegral = 0;
double moveError = 0, lastMoveError = 0, moveIntegral = 0;

/**
 * Calculates PID output for turning to center the object
 * @return Speed adjustment (-100 to 100) for turning
 */
double calculateTurnPID(double errorPixels) {
    turnError = errorPixels;
    
    // Integral term with anti-windup
    turnIntegral += turnError;
    if (turnIntegral > 100) turnIntegral = 100;
    if (turnIntegral < -100) turnIntegral = -100;
    
    // Derivative term
    double derivative = turnError - lastTurnError;
    lastTurnError = turnError;
    
    // Calculate PID output
    double output = (TURN_KP * turnError) + (TURN_KI * turnIntegral) + (TURN_KD * derivative);
    
    // Clamp output
    if (output > MAX_TURN_SPEED) output = MAX_TURN_SPEED;
    if (output < -MAX_TURN_SPEED) output = -MAX_TURN_SPEED;
    
    return output;
}

/**
 * Calculates PID output for moving forward/backward to maintain distance
 * @return Speed adjustment (-100 to 100) for forward/backward motion
 */
double calculateMovePID(double errorPixels) {
    moveError = errorPixels;
    
    // Integral term with anti-windup
    moveIntegral += moveError;
    if (moveIntegral > 100) moveIntegral = 100;
    if (moveIntegral < -100) moveIntegral = -100;
    
    // Derivative term
    double derivative = moveError - lastMoveError;
    lastMoveError = moveError;
    
    // Calculate PID output
    double output = (MOVE_KP * moveError) + (MOVE_KI * moveIntegral) + (MOVE_KD * derivative);
    
    // Clamp output
    if (output > MAX_MOVE_SPEED) output = MAX_MOVE_SPEED;
    if (output < -MAX_MOVE_SPEED) output = -MAX_MOVE_SPEED;
    
    return output;
}

/**
 * Applies motor speeds to the drivetrain
 * @param turnSpeed: Rotational speed (-100 to 100)
 * @param moveSpeed: Forward/backward speed (-100 to 100)
 */
void applyMotorSpeeds(double turnSpeed, double moveSpeed) {
    // Calculate left and right motor speeds
    // turnSpeed > 0: turn right (slow left, speed right)
    // moveSpeed > 0: move forward
    
    double leftSpeed = moveSpeed - turnSpeed;
    double rightSpeed = moveSpeed + turnSpeed;
    
    // Clamp to valid range
    if (leftSpeed > 100) leftSpeed = 100;
    if (leftSpeed < -100) leftSpeed = -100;
    if (rightSpeed > 100) rightSpeed = 100;
    if (rightSpeed < -100) rightSpeed = -100;
    
    // Apply speeds to motor arrays
    for (int i = 0; i < 3; i++) {
        leftMotor[i].spin(directionType::fwd, leftSpeed, velocityUnits::pct);
        rightMotor[i].spin(directionType::fwd, rightSpeed, velocityUnits::pct);
    }
}

/**
 * Checks if a red ball is detected by the AI Vision sensor
 * @return true if red object detected, false otherwise
 */
bool isRedBallDetected() {
    return AIVision20.objectCount > 0;
}

/**
 * Main function to follow a red ball
 * Uses vision sensor to detect red objects and drive toward them
 */
void followRedBall() {
    Brain.Screen.clearScreen();
    Brain.Screen.print("Vision Follow: Red Ball");
    
    while (true) {
        // Take a snapshot with the red cube signature
        AIVision20.takeSnapshot(AIVision20__redCube);
        
        // Check if any red objects are detected
        if (!isRedBallDetected()) {
            // No ball detected - stop and search
            applyMotorSpeeds(0, 0);
            Brain.Screen.clearScreen();
            Brain.Screen.print("No ball detected");
            vex::task::sleep(100);
            continue;
        }
        
        // Get the first (largest) red object from the objects array
        vex::aivision::object redObject = AIVision20.objects[0];
        
        // Get object position and size
        double objectX = redObject.centerX;
        double objectY = redObject.centerY;
        double objectWidth = redObject.width;
        double objectHeight = redObject.height;
        
        // Display information on brain screen
        Brain.Screen.clearScreen();
        Brain.Screen.setFont(monoL);
        Brain.Screen.print("X: %.0f  Y: %.0f", objectX, objectY);
        Brain.Screen.newLine();
        Brain.Screen.print("Size: %.0f x %.0f", objectWidth, objectHeight);
        
        // ===== TURN CONTROL =====
        // Calculate error from screen center X
        double turnErrorPixels = objectX - SCREEN_CENTER_X;
        double turnSpeed = 0;
        
        if ((turnErrorPixels > TURN_DEADZONE) || (turnErrorPixels < -TURN_DEADZONE)) {
            turnSpeed = calculateTurnPID(turnErrorPixels);
        } else {
            turnSpeed = 0;
            turnIntegral = 0;  // Reset integral when in deadzone
        }
        
        // ===== FORWARD/BACKWARD CONTROL =====
        // Calculate error from desired distance
        // Larger object means too close, smaller means too far
        double moveErrorPixels = objectWidth - STOP_DISTANCE;
        double moveSpeed = 0;
        
        if (objectWidth < MIN_OBJECT_SIZE) {
            // Ball is too far away
            moveSpeed = 30;  // Move forward with moderate speed
        } else if (objectWidth > MAX_OBJECT_SIZE) {
            // Ball is too close
            moveSpeed = -30;  // Back away
        } else if ((moveErrorPixels > MOVE_DEADZONE) || (moveErrorPixels < -MOVE_DEADZONE)) {
            moveSpeed = calculateMovePID(moveErrorPixels);
        } else {
            moveSpeed = 0;
            moveIntegral = 0;  // Reset integral when in deadzone
        }
        
        // Apply calculated speeds
        applyMotorSpeeds(turnSpeed, moveSpeed);
        
        Brain.Screen.newLine();
        Brain.Screen.print("Turn Err: %.1f", turnErrorPixels);
        Brain.Screen.newLine();
        Brain.Screen.print("Move Err: %.1f", moveErrorPixels);
        Brain.Screen.newLine();
        Brain.Screen.print("L: %.0f  R: %.0f", 
                          moveSpeed - turnSpeed, 
                          moveSpeed + turnSpeed);
        
        // Small delay to prevent loop from running too fast
        vex::task::sleep(50);
    }
}

/**
 * Stops the robot and halts the following behavior
 */
void stopFollowing() {
    for (int i = 0; i < 3; i++) {
        leftMotor[i].stop();
        rightMotor[i].stop();
    }
    
    // Reset PID variables
    turnError = 0;
    lastTurnError = 0;
    turnIntegral = 0;
    moveError = 0;
    lastMoveError = 0;
    moveIntegral = 0;
    
    Brain.Screen.clearScreen();
}
