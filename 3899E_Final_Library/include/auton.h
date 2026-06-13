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
void sweepTest(bool isRed);
void sweepAndReturn(bool isRed);

// ── BLUE ALLIANCE RIGHT SIDE ──────────────────────────────────────────────────
// Quadrant I start: X positive, Y positive. Heading -90° (facing West/opponent).
// Call blueRightIsolation() from isolation period (15s, stay on east side X > 0).
// Call blueRightInteraction() from interaction period (1m45s, full field sweep).
void blueRightIsolation();
void blueRightInteraction();

// ── RED ALLIANCE RIGHT SIDE ───────────────────────────────────────────────────
// Quadrant II start: X negative, Y positive. Heading 90° (facing East/opponent).
// Call redRightIsolation() from isolation period (15s, stay on west side X < 0).
// Call redRightInteraction() from interaction period (1m45s, full field sweep).
void redRightIsolation();
void redRightInteraction();

// ── SCORING MECHANISM TEST ────────────────────────────────────────────────────
// Tests hood piston, indexers, hood motor, and gate individually then together.
// Use from Slot 6 (dev/test only). isRed=true → left gate, false → right gate.
void testSweepScore(bool isRed);

// Tests intake motors + upper indexer as they run during sweep (5 seconds).
// Hood stays down. Use to verify intake direction before a match.
void testSweepIntake();

#endif // AUTON_H