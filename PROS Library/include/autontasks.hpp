#ifndef AUTONTASKS_HPP
#define AUTONTASKS_HPP

#include "utils.hpp"
#include "main.h" // PROS types
#include <memory> // for std::unique_ptr

// ==========================================
// TASK CONTROLLER (Safe Wrapper)
// ==========================================
// Handles PROS tasks safely without manual pointers.
class TaskController {
public:
    TaskController() = default;
    
    // Starts the task. If one is running, it stops it safely first.
    void start(void(*taskFunc)(void*), const char* name) {
        handle = std::make_unique<pros::Task>(taskFunc, nullptr, name);
    }

    // Stops the task safely.
    void stop() {
        handle.reset(); 
    }

    bool isRunning() const {
        return handle != nullptr;
    }

private:
    std::unique_ptr<pros::Task> handle;
};

// ==========================================
// FUNCTION DECLARATIONS
// ==========================================

// Heading Display Control
void startHeadingDisplay();
void stopHeadingDisplay();

// New tasks
void intakeHopperStart(double timeMs, double power, double delayMs = 0, bool async = true);
void matchloadStart(double timeMs, double power, double delayMs = 0, bool async = true);

// Legacy functions
void intake(double time, bool pistonState);
void intakeStart(double timeMs, double intakePct, bool pistonState);
void intakeStart2(double timeMs, double intakePct, bool pistonState, bool matchLoad, Color targetColor);
void intakeStop();
void score(double time, double power);
void stopScore();
void outtake(double time, double power);
void stopOuttake();
void scoreStart(double timeMs, double power);

// Global Variables
extern double g_targetDistance;
extern double g_targetHeading;

// Task Function Prototype
void headingDisplayTask(void* ignore);

#endif // AUTONTASKS_HPP