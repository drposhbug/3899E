#ifndef AUTON_H
#define AUTON_H

// ══════════════════════════════════════════════════════════════════════════════
// AUTON ROUTINES — declarations for auton.cpp functions
// ══════════════════════════════════════════════════════════════════════════════
void autonSelector();
void runAIMatchRoute();
void routeTest();
void navTest();
void visionTest();
void routeGridTest();
void systemTest();
void coordinateFinder();
void gpsTest();
void fieldTargetsTest();
void rightSideAuton();
void longGoalAuto15s(bool isRedAlliance = false); 
void TestTurn(bool isRedAlliance = false);  // default: blue alliance
void sweepAndScore();

#endif // AUTON_H