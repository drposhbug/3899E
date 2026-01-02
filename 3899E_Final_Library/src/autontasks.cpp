#include "robot_config.h"
#include <atomic> 
#include "utils.h"
#include "vex.h"
#include "navigation.h"
#include "odometry.h"
#include <cmath>

using namespace vex;

//intake only
void intake(double time, bool pistonState) //time in milliseconds, true for pistons, false for no pistons
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

            if (g_intakePistonState) {
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
    ptoPneumatics.set(true); 

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

void colourDetectTest(Color targetColor){
    initializeOpticalSensor();
    resetColorDetection();
    while (true){
        if (detectColor(targetColor)){
            Brain.Screen.printAt(10, 50, targetColor "Detected");
            intakeMotor1.stop();
            intakeMotor2.stop();
        }
        else {
            Brain.Screen.printAt(10, 50, "Not Detected");
            intakeMotor1.spin(forward, 12.0, voltageUnits::volt);
            intakeMotor2.spin(forward, 12.0, voltageUnits::volt);
        }
        vex::task::sleep(100);
    }
}
