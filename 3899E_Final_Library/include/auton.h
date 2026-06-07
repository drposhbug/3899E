#ifndef AUTON_H
#define AUTON_H

// ── VAIRC match routes ────────────────────────────────────────────────────────
void runAIMatchRoute();    // full VAIRC match via Jetson + ai.cpp

// ── Field test / diagnostic routes ───────────────────────────────────────────
void routeTest();          // plan and drive a route manually
void routeGridTest();      // print obstacle grid to brain screen
void systemTest();         // spin each motor to verify hardware
void coordinateFinder();   // live GPS/odometry coordinate display
void navTest();            // test odometry-based straight drive
void visionTest();         // test vision-based drive
void fieldTargetsTest();   // test navigateTo() end-to-end for a named target

// ── Selector — called from autonomous() in main.cpp ──────────────────────────
void autonSelector();

void autonLeft15();
void skills();
#endif // AUTON_H