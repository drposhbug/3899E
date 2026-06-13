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
void colorSortTest();      // test color sort with optical sensor
void fieldTargetsTest();   // test navigateTo() end-to-end for a named target
void redColorSortTest();   // test color sort with red cubes
void blueColorSortTest();  // test color sort with blue cubes

// ── Selector — called from autonomous() in main.cpp ──────────────────────────
void autonSelector();

void autonLeft15();
void leftAuton();
void skills();
void autonLeftAStar(std::pmr::string teamColor);  // Left side with A*, matchload, and long goal score
#endif // AUTON_H