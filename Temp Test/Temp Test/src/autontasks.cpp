#include "main.h"
#include "autontasks.hpp"   // Defines HeadingDisplayParams
#include "robot_config.hpp" // Defines intakeMotor1, pneumatics

/**
 * @file autontasks.cpp
 * @brief Autonomous task management for intake, scoring, and display systems
 * 
 * Provides asynchronous task control for:
 * - Intake operations (hopper, matchload, color detection)
 * - Scoring and outtake mechanisms
 * - Controller heading display
 * 
 * Converted from VEX VCSCode to PROS V4
 */

#include "main.h"
#include <atomic>
#include <cmath>

// ============================================================================
// GLOBAL DISPLAY VARIABLES
// Updated by navigation functions for real-time feedback
// ============================================================================
double g_targetDistance = 0.0;  // Target distance in cm
double g_targetHeading = 0.0;   // Target heading in degrees

// Heading display parameters
HeadingDisplayParams headingDisplayParams = {false};

// ============================================================================
// SHARED ASYNC TASK STRUCTURE
// ============================================================================
struct AsyncTaskParams {
    std::atomic<bool> running{false};
    double timeMs = 0;
    double delayMs = 0;
    double power = 100;
    pros::Task* taskHandle = nullptr;
};

// ============================================================================
// INTAKE HOPPER TASK (motors + hood pneumatics)
// Runs intake motors with front hood closed, back hood open
// ============================================================================
static AsyncTaskParams intakeHopperParams;

void intakeHopperTaskFunc(void*) {
    intakeHopperParams.running.store(true);

    // Initial delay if specified
    if (intakeHopperParams.delayMs > 0) {
        pros::delay(intakeHopperParams.delayMs);
    }

    // Configure hood for intake
    frontHoodPneumatics.set_value(true);   // Close front hood
    backHoodPneumatics.set_value(false);   // Open back hood

    uint32_t startTime = pros::millis();
    double voltage = intakeHopperParams.power / 8.34;

    // Run intake until time expires or task is stopped
    while (intakeHopperParams.running.load() && 
           (pros::millis() - startTime) < intakeHopperParams.timeMs) {
        intakeMotor1.move_voltage(voltage * 1000);  // PROS uses millivolts
        intakeMotor2.move_voltage(voltage * 1000);
        pros::delay(10);
    }

    // Cleanup
    intakeMotor1.brake();
    intakeMotor2.brake();
    intakeHopperParams.running.store(false);
}

/**
 * @brief Start intake hopper operation
 * @param timeMs Duration to run intake (milliseconds)
 * @param power Motor power percentage (0-100)
 * @param delayMs Delay before starting (milliseconds)
 * @param async Run asynchronously if true
 */
void intakeHopperStart(double timeMs, double power, double delayMs, bool async) {
    // Stop existing task if running
    if (intakeHopperParams.running.load()) {
        intakeHopperParams.running.store(false);
        pros::delay(20);
        if (intakeHopperParams.taskHandle != nullptr) {
            delete intakeHopperParams.taskHandle;
            intakeHopperParams.taskHandle = nullptr;
        }
    }
    
    intakeHopperParams.timeMs = timeMs;
    intakeHopperParams.power = power;
    intakeHopperParams.delayMs = delayMs;
    
    if (async) {
        intakeHopperParams.taskHandle = new pros::Task(intakeHopperTaskFunc, nullptr, "IntakeHopper");
    } else {
        intakeHopperTaskFunc(nullptr);
    }
}

// ============================================================================
// MATCHLOAD TASK (motors + hood + matchload pneumatic)
// Extends matchload pneumatic after starting intake, retracts on completion
// ============================================================================
static AsyncTaskParams matchloadParams;
static double matchloadRetractDelay = 200;  // Delay before retracting pneumatic (ms)

void matchloadTaskFunc(void*) {
    matchloadParams.running.store(true);

    // Initial delay if specified
    if (matchloadParams.delayMs > 0) {
        pros::delay(matchloadParams.delayMs);
    }

    // Configure hood for intake
    frontHoodPneumatics.set_value(true);   // Close front hood
    backHoodPneumatics.set_value(false);   // Open back hood

    // Start intake motors
    uint32_t startTime = pros::millis();
    double voltage = matchloadParams.power / 8.34;
    intakeMotor1.move_voltage(voltage * 1000);
    intakeMotor2.move_voltage(voltage * 1000);

    // Extend matchload pneumatic
    matchLoadPneumatics.set_value(true);

    // Run until time expires or task is stopped
    while (matchloadParams.running.load() && 
           (pros::millis() - startTime) < matchloadParams.timeMs) {
        pros::delay(10);
    }

    // Stop intake first
    intakeMotor1.brake();
    intakeMotor2.brake();

    // Delay then retract matchload pneumatic
    pros::delay(matchloadRetractDelay);
    matchLoadPneumatics.set_value(false);

    matchloadParams.running.store(false);
}

/**
 * @brief Start matchload operation
 * @param timeMs Duration to run (milliseconds)
 * @param power Motor power percentage (0-100)
 * @param delayMs Delay before starting (milliseconds)
 * @param async Run asynchronously if true
 */
void matchloadStart(double timeMs, double power, double delayMs, bool async) {
    // Stop existing task if running
    if (matchloadParams.running.load()) {
        matchloadParams.running.store(false);
        pros::delay(20);
        if (matchloadParams.taskHandle != nullptr) {
            delete matchloadParams.taskHandle;
            matchloadParams.taskHandle = nullptr;
        }
    }
    
    matchloadParams.timeMs = timeMs;
    matchloadParams.power = power;
    matchloadParams.delayMs = delayMs;
    
    if (async) {
        matchloadParams.taskHandle = new pros::Task(matchloadTaskFunc, nullptr, "Matchload");
    } else {
        matchloadTaskFunc(nullptr);
    }
}

// ============================================================================
// BASIC INTAKE (synchronous, no color detection)
// ============================================================================

/**
 * @brief Run basic intake for specified duration
 * @param time Duration in milliseconds
 * @param pistonState true = close front/open back, false = both closed
 */
void intake(double time, bool pistonState) {
    uint32_t startTime = pros::millis();

    // Configure pneumatics
    if (pistonState) {
        frontHoodPneumatics.set_value(true);   // Close front hood
        backHoodPneumatics.set_value(false);   // Open back hood
    } else {
        frontHoodPneumatics.set_value(false);
        backHoodPneumatics.set_value(false);
    }

    // Run intake motors
    while ((pros::millis() - startTime) < time) {
        intakeMotor1.move_voltage(12000);  // 12V = 12000 millivolts
        intakeMotor2.move_voltage(12000);
        pros::delay(10);
    }

    // Cleanup
    backHoodPneumatics.set_value(false);
    intakeMotor1.brake();
    intakeMotor2.brake();
}

// ============================================================================
// ASYNCHRONOUS INTAKE WITH COLOR DETECTION
// Detects and ejects wrong-colored rings during intake
// ============================================================================
static std::atomic<bool> g_intakeTaskRunning(false);
static double g_intakeTimeMs = 0;
static double g_intakePct = 100;
static bool g_intakePistonState = false;
static bool g_matchLoadState = false;
static Color g_colourDetectionTarget = Color::RED;
static bool g_enableColourDetection = false;
static pros::Task* g_intakeTaskHandle = nullptr;

void intakeTaskEntry(void*) {
    g_intakeTaskRunning.store(true);

    // Configure matchload pneumatic
    if (g_matchLoadState) {
        matchLoadPneumatics.set_value(true);
    } else {
        matchLoadPneumatics.set_value(false);
    }

    // Configure hood pneumatics
    if (g_intakePistonState) {
        frontHoodPneumatics.set_value(true);
        backHoodPneumatics.set_value(false);
    } else {
        frontHoodPneumatics.set_value(false);
        backHoodPneumatics.set_value(false);
    }

    // Run intake with color detection
    uint32_t startTime = pros::millis();
    double intakeVoltage = g_intakePct / 8.34;
    
    while (g_intakeTaskRunning.load() && 
           (pros::millis() - startTime) < g_intakeTimeMs) {
        
        intakeMotor1.move_voltage(intakeVoltage * 1000);
        intakeMotor2.move_voltage(intakeVoltage * 1000);

        // Color detection and ejection sequence
        if (g_enableColourDetection && detectColor(g_colourDetectionTarget)) {
            // Stop intake
            intakeMotor1.brake();
            intakeMotor2.brake();
            pros::delay(50);

            // Prepare for ejection
            ptoPneumatics.set_value(true);
            frontHoodPneumatics.set_value(false);
            backHoodPneumatics.set_value(false);

            // Eject wrong-colored ring (300ms outtake)
            uint32_t outtakeStart = pros::millis();
            while ((pros::millis() - outtakeStart) < 300) {
                intakeMotor1.move_voltage(12000);
                intakeMotor2.move_voltage(12000);
                pros::delay(10);
            }

            // Stop ejection and reset
            ptoPneumatics.set_value(false);
            intakeMotor1.brake();
            intakeMotor2.brake();
            resetColorDetection();
            pros::delay(50);

            // Resume intake configuration
            if (g_intakePistonState) {
                frontHoodPneumatics.set_value(true);
                backHoodPneumatics.set_value(false);
            } else {
                frontHoodPneumatics.set_value(false);
                backHoodPneumatics.set_value(false);
            }
        }

        pros::delay(10);
    }

    // Cleanup all systems
    intakeMotor1.brake();
    intakeMotor2.brake();
    frontHoodPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    matchLoadPneumatics.set_value(false);
    ptoPneumatics.set_value(false);
    g_intakeTaskRunning.store(false);
}

/**
 * @brief Start asynchronous intake with color detection
 * @param timeMs Duration to run (milliseconds)
 * @param intakePct Motor power percentage (0-100)
 * @param pistonState Hood configuration
 * @param matchLoad Enable matchload pneumatic
 * @param targetColor Color to detect and eject (RED or BLUE)
 */
void intakeStart2(double timeMs, double intakePct, bool pistonState, 
                  bool matchLoad, Color targetColor) {
    // Stop existing task if running
    if (g_intakeTaskRunning.load()) {
        g_intakeTaskRunning.store(false);
        pros::delay(20);
        if (g_intakeTaskHandle != nullptr) {
            delete g_intakeTaskHandle;
            g_intakeTaskHandle = nullptr;
        }
    }
    
    g_intakeTimeMs = timeMs;
    g_intakePistonState = pistonState;
    g_intakePct = intakePct;
    g_matchLoadState = matchLoad;
    g_colourDetectionTarget = targetColor;
    g_enableColourDetection = true;
    resetColorDetection();
    
    g_intakeTaskHandle = new pros::Task(intakeTaskEntry, nullptr, "IntakeColor");
}

/**
 * @brief Start asynchronous intake without color detection
 * @param timeMs Duration to run (milliseconds)
 * @param intakePct Motor power percentage (0-100)
 * @param pistonState Hood configuration
 */
void intakeStart(double timeMs, double intakePct, bool pistonState) {
    // Stop existing task if running
    if (g_intakeTaskRunning.load()) {
        g_intakeTaskRunning.store(false);
        pros::delay(20);
        if (g_intakeTaskHandle != nullptr) {
            delete g_intakeTaskHandle;
            g_intakeTaskHandle = nullptr;
        }
    }
    
    g_intakeTimeMs = timeMs;
    g_intakePistonState = pistonState;
    g_intakePct = intakePct;
    g_enableColourDetection = false;
    
    g_intakeTaskHandle = new pros::Task(intakeTaskEntry, nullptr, "Intake");
}

/**
 * @brief Stop asynchronous intake task early
 */
void intakeStop() {
    g_intakeTaskRunning.store(false);
    pros::delay(20);
}

// ============================================================================
// SCORING SYSTEM (synchronous)
// Opens front hood, closes back hood, engages PTO for scoring
// ============================================================================

/**
 * @brief Run scoring sequence
 * @param time Duration in milliseconds
 * @param power Motor power percentage (0-100)
 */
void score(double time, double power) {
    uint32_t startTime = pros::millis();

    // Configure for scoring
    frontHoodPneumatics.set_value(false);  // Open front hood
    backHoodPneumatics.set_value(true);    // Close back hood
    ptoPneumatics.set_value(true);         // Engage PTO

    double voltagePower = power / 8.34;

    // Run scoring motors
    while ((pros::millis() - startTime) < time) {
        intakeMotor1.move_voltage(voltagePower * 1000);
        intakeMotor2.move_voltage(voltagePower * 1000);
        pros::delay(10);
    }

    // Cleanup
    frontHoodPneumatics.set_value(false);
    intakeMotor1.brake();
    intakeMotor2.brake();
    ptoPneumatics.set_value(false);
}

/**
 * @brief Emergency stop for scoring
 */
void stopScore() {
    intakeMotor1.brake();
    intakeMotor2.brake();
}

// ============================================================================
// OUTTAKE SYSTEM (synchronous)
// Reverses intake with PTO engaged for ring ejection
// ============================================================================

/**
 * @brief Run outtake sequence
 * @param time Duration in milliseconds
 * @param power Motor power percentage (0-100)
 */
void outtake(double time, double power) {
    uint32_t startTime = pros::millis();

    // Configure for outtake
    ptoPneumatics.set_value(true);         // Engage PTO
    frontHoodPneumatics.set_value(false);  // Close front hood
    backHoodPneumatics.set_value(false);   // Close back hood

    // Run outtake motors in reverse
    while ((pros::millis() - startTime) < time) {
        double outtakePower = power / 8.34;
        intakeMotor1.move_voltage(-outtakePower * 1000);  // Negative for reverse
        intakeMotor2.move_voltage(-outtakePower * 1000);
        pros::delay(10);
    }

    // Cleanup
    ptoPneumatics.set_value(false);
    intakeMotor1.brake();
    intakeMotor2.brake();
}

/**
 * @brief Emergency stop for outtake
 */
void stopOuttake() {
    intakeMotor1.brake();
    intakeMotor2.brake();
}

// ============================================================================
// CONTROLLER HEADING DISPLAY TASK
// Displays encoder positions, heading, and target values on controller
// ============================================================================

void headingDisplayTask(void* params) {
    HeadingDisplayParams* p = static_cast<HeadingDisplayParams*>(params);
    
    // SAFETY: Update only once per second to prevent VEXnet radio flood/disconnects.
    while (p->isRunning) {
        // 1. Get Values
        // Uses 'inertialSensor' (lowercase) to match your robot_config.cpp
        double heading = headingOffset - inertialSensor.get_rotation();
        
        // Convert PROS cdeg (centidegrees) to degrees, then to cm
        double leftEnc = passiveEncoderLeft.get_position() / 100.0; 
        double rightEnc = passiveEncoderRight.get_position() / 100.0;
        
        double leftCM = leftEnc * encoderWheelCircumferenceCM / 360.0;
        double rightCM = rightEnc * encoderWheelCircumferenceCM / 360.0;
        double avgCM = (leftCM + rightCM) / 2.0;
        
        // 2. Print to Controller (Line by Line with delays)
        // Uses 'controller' (lowercase) to match your robot_config.cpp
        
        controller.print(0, 0, "L:%.0f R:%.0f   ", leftCM, rightCM);
        pros::delay(60); // Gap for radio packet
        
        controller.print(1, 0, "A:%.0f H:%.1f   ", avgCM, heading);
        pros::delay(60);

        controller.print(2, 0, "T:%.0f H:%.0f   ", g_targetDistance, g_targetHeading);
        
        // 3. LONG WAIT: 1 second total cycle time
        // This keeps the radio channel clear for your driver control packets.
        pros::delay(900); 
    }
}

// ============================================================================
// ASYNCHRONOUS SCORING TASK
// Non-blocking scoring operation
// ============================================================================
static std::atomic<bool> g_scoringTaskRunning(false);
static double g_scoringTimeMs = 0;
static double g_scoringPower = 100;
static pros::Task* g_scoringTaskHandle = nullptr;

void scoringTaskEntry(void*) {
    g_scoringTaskRunning.store(true);

    // Configure for scoring
    frontHoodPneumatics.set_value(false);  // Open front hood
    backHoodPneumatics.set_value(true);    // Close back hood
    ptoPneumatics.set_value(true);         // Engage ptoPnuematics

    uint32_t startTime = pros::millis();
    double voltagePower = g_scoringPower / 8.34;

    // Run scoring motors
    while (g_scoringTaskRunning.load() && 
           (pros::millis() - startTime) < g_scoringTimeMs) {
        intakeMotor1.move_voltage(voltagePower * 1000);
        intakeMotor2.move_voltage(voltagePower * 1000);
        pros::delay(10);
    }

    // Cleanup
    frontHoodPneumatics.set_value(false);
    intakeMotor1.brake();
    intakeMotor2.brake();
    ptoPneumatics.set_value(false);
    g_scoringTaskRunning.store(false);
}

/**
 * @brief Start asynchronous scoring operation
 * @param timeMs Duration to run (milliseconds)
 * @param power Motor power percentage (0-100)
 */
void scoreStart(double timeMs, double power) {
    // Stop existing task if running
    if (g_scoringTaskRunning.load()) {
        g_scoringTaskRunning.store(false);
        pros::delay(20);
        if (g_scoringTaskHandle != nullptr) {
            delete g_scoringTaskHandle;
            g_scoringTaskHandle = nullptr;
        }
    }
    
    g_scoringTimeMs = timeMs;
    g_scoringPower = power;
    
    g_scoringTaskHandle = new pros::Task(scoringTaskEntry, nullptr, "ScoringAsync");
}