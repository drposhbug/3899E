#include "robot_config.h"
#include <atomic> 
#include "utils.h"
#include "vex.h"
#include "odometry.h"
#include "autontasks.h"
#include <cmath>

using namespace vex;

// Heading display task for controller

HeadingDisplayParams headingDisplayParams = {false};

// Global variables for display - navigation functions update these
double g_targetDistance = 0.0;  // tgtD - target distance in cm
double g_targetHeading = 0.0;   // tgtH - target heading in degrees

// ===== SHARED TASK STRUCTURE =====
struct AsyncTaskParams {
    std::atomic<bool> running{false};
    double timeMs = 0;
    double delayMs = 0;
    double power = 100;
    vex::task handle;
};

// ===== INTAKE HOPPER TASK (motors + hood) =====
static AsyncTaskParams intakeHopperParams;

int intakeHopperTask(void*) {
    intakeHopperParams.running.store(true);

    if (intakeHopperParams.delayMs > 0) {
        vex::task::sleep(intakeHopperParams.delayMs);
    }

    // Set hood positions for intake
    frontHoodPneumatics.set(true);   // Close front hood
    backHoodPneumatics.set(false);   // Open back hood

    vex::timer t;
    double voltage = intakeHopperParams.power / 8.34;

    while (intakeHopperParams.running.load() && t.time(msec) < intakeHopperParams.timeMs) {
        intakeMotor1.spin(forward, voltage, voltageUnits::volt);
        intakeMotor2.spin(forward, voltage, voltageUnits::volt);
        vex::task::sleep(10);
    }

    intakeMotor1.stop();
    intakeMotor2.stop();
    intakeHopperParams.running.store(false);
    return 0;
}

void intakeHopperStart(double timeMs, double power, double delayMs, bool async) {
    if (intakeHopperParams.running.load()) {
        intakeHopperParams.running.store(false);
        vex::task::sleep(20);
    }
    intakeHopperParams.timeMs = timeMs;
    intakeHopperParams.power = power;
    intakeHopperParams.delayMs = delayMs;
    
    if (async) {
        intakeHopperParams.handle = vex::task(intakeHopperTask, nullptr);
    } else {
        intakeHopperTask(nullptr);
    }
}


// ===== MATCHLOAD TASK (motors + hood + matchload pneumatic) =====
static AsyncTaskParams matchloadParams;
static double matchloadRetractDelay = 200;  // ms delay before retracting pneumatic

int matchloadTask(void*) {
    matchloadParams.running.store(true);

    if (matchloadParams.delayMs > 0) {
        vex::task::sleep(matchloadParams.delayMs);
    }

    // Set hood positions for intake
    frontHoodPneumatics.set(true);   // Close front hood
    backHoodPneumatics.set(false);   // Open back hood

    // Start intake
    vex::timer t;
    double voltage = matchloadParams.power / 8.34;
    intakeMotor1.spin(forward, voltage, voltageUnits::volt);
    intakeMotor2.spin(forward, voltage, voltageUnits::volt);

    // Then extend matchload pneumatic
    matchLoadPneumatics.set(true);

    while (matchloadParams.running.load() && t.time(msec) < matchloadParams.timeMs) {
        vex::task::sleep(10);
    }

    // Stop intake first
    intakeMotor1.stop();
    intakeMotor2.stop();

    // Delay then retract pneumatic
    vex::task::sleep(matchloadRetractDelay);
    matchLoadPneumatics.set(false);

    matchloadParams.running.store(false);
    return 0;
}

void matchloadStart(double timeMs, double power, double delayMs, bool async) {
    if (matchloadParams.running.load()) {
        matchloadParams.running.store(false);
        vex::task::sleep(20);
    }
    matchloadParams.timeMs = timeMs;
    matchloadParams.power = power;
    matchloadParams.delayMs = delayMs;
    
    if (async) {
        matchloadParams.handle = vex::task(matchloadTask, nullptr);
    } else {
        matchloadTask(nullptr);
    }
}



//intake only
void intake(double time, bool pistonState) //time in milliseconds, true for pistons, false for no pistons
{
    vex::timer intakeTime;
    intakeTime.reset();

    if (pistonState == true){
        frontHoodPneumatics.set(true); //opebn front hood
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
    intakeMotor1.stop();
    intakeMotor2.stop();
}

//asynchronous intake
static std::atomic<bool> g_intakeTaskRunning(false);
static double g_intakeTimeMs = 0;
static double g_intakePct = 100;
static bool g_intakePistonState = false;
static bool g_matchLoadState = false;
static Color g_colourDetectionTarget = Color::RED; // colour to detect defaults to RED
static bool g_enableColourDetection = false;
static vex::task g_intakeTaskHandle;

int intakeTaskEntry(void*) {
    g_intakeTaskRunning.store(true);

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
        intakeMotor1.spin(forward, intakeVoltage, voltageUnits::volt);
        intakeMotor2.spin(forward, intakeVoltage, voltageUnits::volt);

        // check for colour detection if enabled
        if (g_enableColourDetection && detectColor(g_colourDetectionTarget)) {
            // stop intake
            intakeMotor1.stop();
            intakeMotor2.stop();
            vex::task::sleep(50);

            // outtake 300ms
            ptoPneumatics.set(true);
            frontHoodPneumatics.set(false);
            backHoodPneumatics.set(false);

            vex::timer outtakeTimer;
            outtakeTimer.reset();
            while (outtakeTimer.time(timeUnits::msec) < 300) {
                intakeMotor1.spin(forward, 12.0, voltageUnits::volt);
                intakeMotor2.spin(forward, 12.0, voltageUnits::volt);
                vex::task::sleep(10);
            }

            ptoPneumatics.set(false);
            intakeMotor1.stop();
            intakeMotor2.stop();

            // Reset colour detection and resume intake
            resetColorDetection();
            vex::task::sleep(50);

            if (g_intakePistonState = true) {
                frontHoodPneumatics.set(true);
                backHoodPneumatics.set(false);
            } else {
                frontHoodPneumatics.set(false);
                backHoodPneumatics.set(false);
            }
        }

        vex::task::sleep(10);
    }

    //cleanup
    intakeMotor1.stop();
    intakeMotor2.stop();
    frontHoodPneumatics.set(false);
    backHoodPneumatics.set(false);
    matchLoadPneumatics.set(false);
    ptoPneumatics.set(false);
    g_intakeTaskRunning.store(false);

    return 0;
}

//asynchronous intake with colour detection
void intakeStart2(double timeMs, double intakePct, bool pistonState, bool matchLoad, Color targetColor) {
    // if already running, stop previous then start new
    if (g_intakeTaskRunning.load()) {   
        g_intakeTaskRunning.store(false);
        vex::task::sleep(20);
    }
    g_intakeTimeMs = timeMs;
    g_intakePistonState = pistonState;
    g_intakePct = intakePct;
    g_matchLoadState = matchLoad;
    g_colourDetectionTarget = targetColor;
    g_enableColourDetection = true;
    resetColorDetection();
    g_intakeTaskHandle = vex::task(intakeTaskEntry, nullptr);
}

void intakeStart(double timeMs, double intakePct, bool pistonState) {
    // if already running, stop previous then start new
    if (g_intakeTaskRunning.load()) {   
        g_intakeTaskRunning.store(false);
        vex::task::sleep(20);
    }
    g_intakeTimeMs = timeMs;
    g_intakePistonState = pistonState;
    g_intakePct = intakePct;
    //g_matchLoadState = matchLoad;
    g_intakeTaskHandle = vex::task(intakeTaskEntry, nullptr);
}



//stop the async intake task early
void intakeStop() {
    g_intakeTaskRunning.store(false);
    vex::task::sleep(20);
}

void score(double time, double power) //skibidi score
{
    vex::timer scoringTime;
    scoringTime.reset();

    frontHoodPneumatics.set(false); //open front hood and close back hood
    backHoodPneumatics.set(true);
    ptoPneumatics.set(true); //engage pto for scoring
    indexPneumatics.set(true);

    double voltagePower = (power / 8.34); //convert power percentage to voltage

    //(scoringTime.time(timeUnits::msec) < time) voltagePower -= time * 0.006;
    while (scoringTime.time(timeUnits::msec) < time)
    {  
        intakeMotor1.spin(forward, voltagePower, voltageUnits::volt);
        intakeMotor2.spin(forward, voltagePower, voltageUnits::volt);
        vex::task::sleep(10);
    }
    frontHoodPneumatics.set(false); //close back hood after scoring
    intakeMotor1.stop();
    intakeMotor2.stop();
    ptoPneumatics.set(false); //disengage pto after scoring
    indexPneumatics.set(false);
}

void stopScore(){
    intakeMotor1.stop();
    intakeMotor2.stop();
}

void outtake(double time, double power) //skibidi outtake
{
    vex::timer outtakeTime;
    outtakeTime.reset();

    ptoPneumatics.set(true); //engage pto for outtake
    frontHoodPneumatics.set(false); //close front hood for outtake
    backHoodPneumatics.set(false); //close back hood for outtake

    while (outtakeTime.time(timeUnits::msec) < time)
    {
        double outtakePower = (power / 8.34);
        intakeMotor1.spin(reverse, outtakePower, voltageUnits::volt);
        intakeMotor2.spin(reverse, outtakePower, voltageUnits::volt);
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

int headingDisplayTask(void *params) {
    HeadingDisplayParams *p = static_cast<HeadingDisplayParams *>(params);
    
    while (p->isRunning) {
        // 1. Read Sensors
        double heading = getAdjustedRotation(); 
        double leftEnc = passiveEncoderLeft.position(rotationUnits::deg);
        double rightEnc = passiveEncoderRight.position(rotationUnits::deg);
        // VERIFY NAME: Check your robot_config.h for the exact name of your X/Center/Aux encoder
        double centerEnc = passiveEncoderX.position(rotationUnits::deg); 
        
        // 2. Convert to CM
        double leftCM = leftEnc * encoderWheelCircumferenceCM / 360.0;
        double rightCM = rightEnc * encoderWheelCircumferenceCM / 360.0;
        double centerCM = centerEnc * encoderWheelCircumferenceCM / 360.0;
        double avgCM = (leftCM + rightCM) / 2.0;
        
        // 3. Print to Controller (Optimized Layout)
        
        // Line 1: Encoders (L=Left, R=Right, X=Center)
        Controller.Screen.setCursor(1, 1);
        // using %.0f removes decimals to save screen space and make it readable
        Controller.Screen.print("L:%.0f R:%.0f X:%.0f  ", leftCM, rightCM, centerCM);
        
        // Line 2: Global Averages
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Avg:%.0f  H:%.1f   ", avgCM, heading);
        
        // Line 3: Targets
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Tgt D:%.0f H:%.0f   ", g_targetDistance, g_targetHeading);
        
        // 4. SAFETY DELAY
        // 250ms = 4 updates/sec. 
        // Anything faster than 100ms risks disconnecting VEXnet during comps.
        wait(500, msec);
    }
    
    Controller.Screen.clearScreen();
    return 0;
}

//asynchronous scoring
static std::atomic<bool> g_scoringTaskRunning(false);
static double g_scoringTimeMs = 0;
static double g_scoringPower = 100;
static vex::task g_scoringTaskHandle;
int scoringTaskEntry(void*) {
    g_scoringTaskRunning.store(true);

    frontHoodPneumatics.set(false); //open front hood and close back hood
    backHoodPneumatics.set(true);
    ptoPneumatics.set(true); //engage pto for scoring

    vex::timer t;
    t.reset();
    double voltagePower = g_scoringPower / 8.34;

    while (g_scoringTaskRunning.load() && t.time(timeUnits::msec) < g_scoringTimeMs) {
        intakeMotor1.spin(forward, voltagePower, voltageUnits::volt);
        intakeMotor2.spin(forward, voltagePower, voltageUnits::volt);
        vex::task::sleep(10);
    }

    frontHoodPneumatics.set(false); //close back hood after scoring
    intakeMotor1.stop();
    intakeMotor2.stop();
    ptoPneumatics.set(false);
    g_scoringTaskRunning.store(false);

    return 0;
}

void scoreStart(double timeMs, double power) {
    if (g_scoringTaskRunning.load()) {   
        g_scoringTaskRunning.store(false);
        vex::task::sleep(20);
    }
    g_scoringTimeMs = timeMs;
    g_scoringPower = power;
    g_scoringTaskHandle = vex::task(scoringTaskEntry, nullptr);
}


//async intake with matchload
static std::atomic<bool> g_matchLoadIntakeTaskRunning(false);
static double g_matchLoadIntakeTimeMs = 0;
static double g_matchLoadIntakePower = 100;
static bool g_matchLoadIntakeMatchload = false;
static double g_matchLoadIntakeDelayMs = 0;
int matchLoadIntakeTaskEntry(void*) {
    g_matchLoadIntakeTaskRunning.store(true);

    if (g_matchLoadIntakeDelayMs > 0) {
        vex::task::sleep(g_matchLoadIntakeDelayMs);
    }

    frontHoodPneumatics.set(true);  
    backHoodPneumatics.set(false);   

    vex::timer t;
    double voltage = g_matchLoadIntakePower / 8.34;
    intakeMotor1.spin(forward, voltage, voltageUnits::volt);
    intakeMotor2.spin(forward, voltage, voltageUnits::volt);


    if (g_matchLoadIntakeMatchload) {
        vex::task::sleep(matchloadRetractDelay);
        matchLoadPneumatics.set(true);
    }

    while (g_matchLoadIntakeTaskRunning.load() && t.time(msec) < g_matchLoadIntakeTimeMs) {
        vex::task::sleep(10);
    }

    // Stop intake first
    intakeMotor1.stop();
    intakeMotor2.stop();

    // Delay then retract pneumatic if it was extended
    if (g_matchLoadIntakeMatchload) {
        matchLoadPneumatics.set(false);
    }

    g_matchLoadIntakeTaskRunning.store(false);
    return 0;
}

void matchLoadIntake(double timeMs, double power, bool matchload, double matchloadDelay){

}


// ======================================================================
// COORDINATE FINDER TASK
// Displays real-time position for path planning
// ======================================================================

// Global task control
struct CoordinateFinderParams {
    bool isRunning;
};

CoordinateFinderParams coordFinderParams = {false};

/**
 * Background task that continuously displays robot position
 * Updates every 500ms and overrides all other screen output
 * Use this to manually push robot around and record coordinates
 */
int coordinateFinderTask(void *params) {
    CoordinateFinderParams *p = static_cast<CoordinateFinderParams *>(params);
    
    while (p->isRunning) {
        // Update odometry
        updateOdometry();
        
        // Clear screen and display position prominently
        Brain.Screen.clearScreen();
        Brain.Screen.setPenColor(vex::color::white);
        
        // Title
        Brain.Screen.setFont(vex::fontType::mono20);
        Brain.Screen.printAt(10, 30, "=== COORDINATE FINDER ===");
        
        // Position data (large font)
        Brain.Screen.setFont(vex::fontType::mono40);
        Brain.Screen.printAt(10, 80, "X: %.1f cm", globalY);
        Brain.Screen.printAt(10, 130, "Y: %.1f cm", globalX);
        
        // Heading (medium font)
        Brain.Screen.setFont(vex::fontType::mono30);
        Brain.Screen.printAt(10, 180, "H: %.1f deg", getNormalizedModifiedHeading());
        
        // Instructions (small font)
        Brain.Screen.setFont(vex::fontType::mono15);
        Brain.Screen.setPenColor(vex::color::yellow);
        Brain.Screen.printAt(10, 220, "Push robot to target position");
        Brain.Screen.printAt(10, 240, "Record coordinates above");
        
        // Wait 500ms before next update
        wait(500, msec);
    }
    
    return 0;
}

/**
 * Start the coordinate finder task
 * Call this after setStartPosition() to begin tracking
 */
void startCoordinateFinder() {
    if (!coordFinderParams.isRunning) {
        coordFinderParams.isRunning = true;
        task coordTask(coordinateFinderTask, &coordFinderParams);
    }
}

/**
 * Stop the coordinate finder task
 * Call this to return control to normal screen output
 */
void stopCoordinateFinder() {
    coordFinderParams.isRunning = false;
    wait(600, msec);  // Wait for task to finish current cycle
    Brain.Screen.clearScreen();
}
