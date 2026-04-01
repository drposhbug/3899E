#include "vision_follow.h"
#include "robot_config.h"

using namespace VisionFollow;

// ── PID state variables ───────────────────────────────────────────────────────
double turnError = 0, lastTurnError = 0, turnIntegral = 0;
double moveError = 0, lastMoveError = 0, moveIntegral = 0;

// ── Turn PID ─────────────────────────────────────────────────────────────────
// Computes turn speed to horizontally center the detected object on screen.
// Returns a speed adjustment in the range [-MAX_TURN_SPEED, +MAX_TURN_SPEED].
double calculateTurnPID(double errorPixels) {
    turnError = errorPixels;

    // Integral with anti-windup clamp
    turnIntegral += turnError;
    if (turnIntegral >  100) turnIntegral =  100;
    if (turnIntegral < -100) turnIntegral = -100;

    // Derivative: rate of change of error
    double derivative = turnError - lastTurnError;
    lastTurnError = turnError;

    double output = (TURN_KP * turnError) + (TURN_KI * turnIntegral) + (TURN_KD * derivative);

    // Clamp output to speed limit
    if (output >  MAX_TURN_SPEED) output =  MAX_TURN_SPEED;
    if (output < -MAX_TURN_SPEED) output = -MAX_TURN_SPEED;

    return output;
}

// ── Move PID ─────────────────────────────────────────────────────────────────
// Computes forward/backward speed to maintain the target object pixel-width.
// Returns a speed adjustment in the range [-MAX_MOVE_SPEED, +MAX_MOVE_SPEED].
double calculateMovePID(double errorPixels) {
    moveError = errorPixels;

    // Integral with anti-windup clamp
    moveIntegral += moveError;
    if (moveIntegral >  100) moveIntegral =  100;
    if (moveIntegral < -100) moveIntegral = -100;

    // Derivative: rate of change of error
    double derivative = moveError - lastMoveError;
    lastMoveError = moveError;

    double output = (MOVE_KP * moveError) + (MOVE_KI * moveIntegral) + (MOVE_KD * derivative);

    // Clamp output to speed limit
    if (output >  MAX_MOVE_SPEED) output =  MAX_MOVE_SPEED;
    if (output < -MAX_MOVE_SPEED) output = -MAX_MOVE_SPEED;

    return output;
}

// ── Motor speed application ───────────────────────────────────────────────────
// Mixes turn and move speeds into left/right drive voltages.
// turnSpeed > 0: turn right (left faster, right slower).
// moveSpeed > 0: move forward.
void applyMotorSpeeds(double turnSpeed, double moveSpeed) {
    double leftSpeed  = moveSpeed - turnSpeed;
    double rightSpeed = moveSpeed + turnSpeed;

    // Clamp to [-100, 100] pct before converting to millivolts
    if (leftSpeed  >  100) leftSpeed  =  100;
    if (leftSpeed  < -100) leftSpeed  = -100;
    if (rightSpeed >  100) rightSpeed =  100;
    if (rightSpeed < -100) rightSpeed = -100;

    // MotorGroup applies the command to all three motors per side simultaneously.
    // Multiply by 120 to convert [-100, 100] pct → [-12000, 12000] mV.
    leftDrive.move_voltage(static_cast<int32_t>(leftSpeed  * 120.0));
    rightDrive.move_voltage(static_cast<int32_t>(rightSpeed * 120.0));
}

// ── Detection check ───────────────────────────────────────────────────────────
// Returns true if the Vision sensor currently sees at least one object.
bool isRedBallDetected() {
    return aiVision.get_object_count() > 0;
}

// ── Main follow loop ──────────────────────────────────────────────────────────
// Continuously reads the Vision sensor for the red-cube signature, then drives
// the robot to keep the object centered horizontally and at STOP_DISTANCE width.
void followRedBall() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Vision Follow: Red Ball");

    while (true) {
        // Get the largest object matching the red-cube signature (index 0 = biggest).
        // PROS Vision sensor is live — no separate takeSnapshot step needed.
        pros::vision_object_s_t redObject = aiVision.get_by_sig(0, aiVision_redCube.id);

        // signature == VISION_OBJECT_ERR_SIG means no match was found
        if (redObject.signature == VISION_OBJECT_ERR_SIG) {
            applyMotorSpeeds(0, 0);  // stop while searching
            pros::screen::erase();
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "No ball detected");
            pros::delay(100);
            continue;
        }

        // Extract position and size from the detected object
        double objectX      = redObject.x_middle_coord;  // horizontal center (pixels)
        double objectY      = redObject.y_middle_coord;  // vertical center (pixels)
        double objectWidth  = redObject.width;
        double objectHeight = redObject.height;

        // Display live detection info on the Brain screen
        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "X: %.0f  Y: %.0f",   objectX,     objectY);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Size: %.0f x %.0f",  objectWidth, objectHeight);

        // ── TURN CONTROL ─────────────────────────────────────────────────────
        // Error = horizontal offset from screen center; positive = object is right of center.
        double turnErrorPixels = objectX - SCREEN_CENTER_X;
        double turnSpeed = 0;

        if ((turnErrorPixels > TURN_DEADZONE) || (turnErrorPixels < -TURN_DEADZONE)) {
            turnSpeed = calculateTurnPID(turnErrorPixels);
        } else {
            turnSpeed    = 0;
            turnIntegral = 0;  // reset integral when inside deadzone to prevent windup
        }

        // ── FORWARD/BACKWARD CONTROL ─────────────────────────────────────────
        // Error = difference between current width and target width (STOP_DISTANCE).
        // Larger width → too close; smaller width → too far.
        double moveErrorPixels = objectWidth - STOP_DISTANCE;
        double moveSpeed = 0;

        if (objectWidth < MIN_OBJECT_SIZE) {
            moveSpeed = 30;   // ball too far — move forward at moderate speed
        } else if (objectWidth > MAX_OBJECT_SIZE) {
            moveSpeed = -30;  // ball too close — back away
        } else if ((moveErrorPixels > MOVE_DEADZONE) || (moveErrorPixels < -MOVE_DEADZONE)) {
            moveSpeed = calculateMovePID(moveErrorPixels);
        } else {
            moveSpeed    = 0;
            moveIntegral = 0;  // reset integral when inside deadzone
        }

        applyMotorSpeeds(turnSpeed, moveSpeed);

        // Display PID debug values
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Turn Err: %.1f", turnErrorPixels);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Move Err: %.1f", moveErrorPixels);
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "L: %.0f  R: %.0f",
                            moveSpeed - turnSpeed,
                            moveSpeed + turnSpeed);

        pros::delay(50);  // 50 ms → 20 Hz control loop
    }
}

// ── Stop ─────────────────────────────────────────────────────────────────────
// Halts both drive sides and resets all PID state.
void stopFollowing() {
    // MotorGroup move(0) respects the configured brake mode for all motors in the group
    leftDrive.move(0);
    rightDrive.move(0);

    // Reset PID state so the next followRedBall() call starts clean
    turnError    = 0;
    lastTurnError = 0;
    turnIntegral = 0;
    moveError    = 0;
    lastMoveError = 0;
    moveIntegral = 0;

    pros::screen::erase();
}
