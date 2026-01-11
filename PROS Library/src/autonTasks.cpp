#include "robot_config.hpp"
#include <atomic> 
#include "utils.hpp"
#include "navigation.hpp"
#include "odometry.hpp"
#include "autontasks.hpp"
#include <cmath>
#include "main.h" 

// Global variables for display
double g_targetDistance = 0.0;
double g_targetHeading = 0.0;

// Shared parameters structure
struct AsyncTaskParams {
    std::atomic<bool> running{false};
    double timeMs = 0;
    double delayMs = 0;
    double power = 100;
};

// ==========================================================
// HEADING DISPLAY TASK (Managed via Wrapper)
// ==========================================================
static TaskController headingController; 
// We no longer need the 'HeadingDisplayParams' struct because the 
// controller handles the start/stop state automatically.

void headingDisplayTask(void* ignore) {
    while (true) { // Run continuously until stopped by controller
        double heading = headingOffset - inertialSensor.get_rotation();
        double leftEnc = passiveEncoderLeft.get_position() / 100.0;
        double rightEnc = passiveEncoderRight.get_position() / 100.0;
        
        double leftCM = leftEnc * encoderWheelCircumferenceCM / 360.0;
        double rightCM = rightEnc * encoderWheelCircumferenceCM / 360.0;
        double avgCM = (leftCM + rightCM) / 2.0;
        
        controller.print(0, 0, "L:%.0f R:%.0f", leftCM, rightCM);
        pros::delay(50);
        controller.print(1, 0, "A:%.0f H:%.1f", avgCM, heading);
        pros::delay(50);
        controller.print(2, 0, "D:%.0f H:%.0f", g_targetDistance, g_targetHeading);
        
        pros::delay(50);
    }
}

void startHeadingDisplay() {
    headingController.start(headingDisplayTask, "HeadingDisplay");
}

void stopHeadingDisplay() {
    headingController.stop();
    controller.clear();
}

// ==========================================================
// INTAKE HOPPER TASK
// ==========================================================
static AsyncTaskParams intakeHopperParams;
static TaskController intakeHopperController; 

void intakeHopperTask(void* ignore) {
    intakeHopperParams.running.store(true);

    if (intakeHopperParams.delayMs > 0) {
        pros::delay(intakeHopperParams.delayMs);
    }

    frontHoodPneumatics.set_value(true);   
    backHoodPneumatics.set_value(false);   

    uint32_t startTime = pros::millis();
    double voltage = (intakeHopperParams.power / 100.0) * 12000; 

    while (intakeHopperParams.running.load() && (pros::millis() - startTime) < intakeHopperParams.timeMs) {
        intakeMotors.move_voltage(voltage);
        pros::delay(10);
    }

    intakeMotors.move_voltage(0);
    intakeHopperParams.running.store(false);
}

void intakeHopperStart(double timeMs, double power, double delayMs, bool async) {
    if (intakeHopperParams.running.load()) {
        intakeHopperParams.running.store(false);
        intakeHopperController.stop(); 
        pros::delay(20);
    }

    intakeHopperParams.timeMs = timeMs;
    intakeHopperParams.power = power;
    intakeHopperParams.delayMs = delayMs;
    
    if (async) {
        intakeHopperController.start(intakeHopperTask, "IntakeHopper");
    } else {
        intakeHopperTask(NULL);
    }
}

// ==========================================================
// MATCHLOAD TASK
// ==========================================================
static AsyncTaskParams matchloadParams;
static TaskController matchloadController;
static double matchloadRetractDelay = 200;

void matchloadTask(void* ignore) {
    matchloadParams.running.store(true);

    if (matchloadParams.delayMs > 0) {
        pros::delay(matchloadParams.delayMs);
    }

    frontHoodPneumatics.set_value(true);
    backHoodPneumatics.set_value(false);

    uint32_t startTime = pros::millis();
    double voltage = (matchloadParams.power / 100.0) * 12000;
    intakeMotors.move_voltage(voltage);

    matchLoadPneumatics.set_value(true);

    while (matchloadParams.running.load() && (pros::millis() - startTime) < matchloadParams.timeMs) {
        pros::delay(10);
    }

    intakeMotors.move_voltage(0);
    pros::delay(matchloadRetractDelay);
    matchLoadPneumatics.set_value(false);

    matchloadParams.running.store(false);
}

void matchloadStart(double timeMs, double power, double delayMs, bool async) {
    if (matchloadParams.running.load()) {
        matchloadParams.running.store(false);
        matchloadController.stop();
        pros::delay(20);
    }
    matchloadParams.timeMs = timeMs;
    matchloadParams.power = power;
    matchloadParams.delayMs = delayMs;
    
    if (async) {
        matchloadController.start(matchloadTask, "Matchload");
    } else {
        matchloadTask(NULL);
    }
}

// Synchronous Intake
void intake(double time, bool pistonState) {
    uint32_t startTime = pros::millis();

    if (pistonState) {
        frontHoodPneumatics.set_value(true);
        backHoodPneumatics.set_value(false);
    } else {
        frontHoodPneumatics.set_value(false);
        backHoodPneumatics.set_value(false);
    }

    while ((pros::millis() - startTime) < time) {
        intakeMotors.move_voltage(-12000); 
        pros::delay(10);
    }
    backHoodPneumatics.set_value(false);
    intakeMotors.brake();
}

// ==========================================================
// ASYNC INTAKE MANAGER
// ==========================================================
static std::atomic<bool> g_intakeTaskRunning(false);
static double g_intakeTimeMs = 0;
static double g_intakePct = 100;
static bool g_intakePistonState = false;
static bool g_matchLoadState = false;
static Color g_colourDetectionTarget = Color::RED;
static bool g_enableColourDetection = false;
static TaskController intakeController; 

void intakeTaskEntry(void* ignore) {
    g_intakeTaskRunning.store(true);

    matchLoadPneumatics.set_value(g_matchLoadState);

    if (g_intakePistonState) {
        frontHoodPneumatics.set_value(true);
        backHoodPneumatics.set_value(false);
    } else {
        frontHoodPneumatics.set_value(false);
        backHoodPneumatics.set_value(false);
    }

    uint32_t startTime = pros::millis();
    double intakeVoltage = (g_intakePct / 100.0) * 12000;

    while (g_intakeTaskRunning.load() && (pros::millis() - startTime) < g_intakeTimeMs) {
        intakeMotors.move_voltage(intakeVoltage);

        if (g_enableColourDetection && detectColor(g_colourDetectionTarget)) {
            intakeMotors.brake();
            pros::delay(50);

            // Outtake 300ms
            ptoPneumatics.set_value(true);
            frontHoodPneumatics.set_value(false);
            backHoodPneumatics.set_value(false);

            uint32_t outtakeStart = pros::millis();
            while ((pros::millis() - outtakeStart) < 300) {
                intakeMotors.move_voltage(12000); 
                pros::delay(10);
            }

            ptoPneumatics.set_value(false);
            intakeMotors.brake();

            resetColorDetection();
            pros::delay(50);

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

    intakeMotors.brake();
    frontHoodPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);
    matchLoadPneumatics.set_value(false);
    ptoPneumatics.set_value(false);
    g_intakeTaskRunning.store(false);
}

void intakeStart2(double timeMs, double intakePct, bool pistonState, bool matchLoad, Color targetColor) {
    if (g_intakeTaskRunning.load()) {   
        g_intakeTaskRunning.store(false);
        intakeController.stop();
        pros::delay(20);
    }
    g_intakeTimeMs = timeMs;
    g_intakePistonState = pistonState;
    g_intakePct = intakePct;
    g_matchLoadState = matchLoad;
    g_colourDetectionTarget = targetColor;
    g_enableColourDetection = true;
    resetColorDetection();
    
    intakeController.start(intakeTaskEntry, "IntakeAsync");
}

void intakeStart(double timeMs, double intakePct, bool pistonState) {
    if (g_intakeTaskRunning.load()) {   
        g_intakeTaskRunning.store(false);
        intakeController.stop();
        pros::delay(20);
    }
    g_intakeTimeMs = timeMs;
    g_intakePistonState = pistonState;
    g_intakePct = intakePct;
    g_enableColourDetection = false;
    
    intakeController.start(intakeTaskEntry, "IntakeAsync");
}

void intakeStop() {
    g_intakeTaskRunning.store(false);
    pros::delay(20);
}

void score(double time, double power) {
    uint32_t startTime = pros::millis();

    frontHoodPneumatics.set_value(false);
    backHoodPneumatics.set_value(true);
    ptoPneumatics.set_value(true);

    double voltage = (power / 100.0) * 12000;

    while ((pros::millis() - startTime) < time) {
        intakeMotors.move_voltage(voltage);
        pros::delay(10);
    }
    frontHoodPneumatics.set_value(false);
    intakeMotors.brake();
    ptoPneumatics.set_value(false);
}

void stopScore(){
    intakeMotors.brake();
}

void outtake(double time, double power) {
    uint32_t startTime = pros::millis();

    ptoPneumatics.set_value(true);
    frontHoodPneumatics.set_value(false);
    backHoodPneumatics.set_value(false);

    double voltage = (power / 100.0) * 12000; 

    while ((pros::millis() - startTime) < time) {
        intakeMotors.move_voltage(-voltage); 
        pros::delay(10);
    }
    ptoPneumatics.set_value(false);
    intakeMotors.brake();
}

void stopOuttake(){
    intakeMotors.brake();
}

// ==========================================================
// SCORING TASK MANAGER
// ==========================================================
static std::atomic<bool> g_scoringTaskRunning(false);
static double g_scoringTimeMs = 0;
static double g_scoringPower = 100;
static TaskController scoringController; 

void scoringTaskEntry(void* ignore) {
    g_scoringTaskRunning.store(true);

    frontHoodPneumatics.set_value(false);
    backHoodPneumatics.set_value(true);
    ptoPneumatics.set_value(true);

    uint32_t startTime = pros::millis();
    double voltage = (g_scoringPower / 100.0) * 12000;

    while (g_scoringTaskRunning.load() && (pros::millis() - startTime) < g_scoringTimeMs) {
        intakeMotors.move_voltage(voltage);
        pros::delay(10);
    }

    frontHoodPneumatics.set_value(false);
    intakeMotors.brake();
    ptoPneumatics.set_value(false);
    g_scoringTaskRunning.store(false);
}

void scoreStart(double timeMs, double power) {
    if (g_scoringTaskRunning.load()) {   
        g_scoringTaskRunning.store(false);
        scoringController.stop();
        pros::delay(20);
    }
    g_scoringTimeMs = timeMs;
    g_scoringPower = power;
    
    scoringController.start(scoringTaskEntry, "ScoreAsync");
}