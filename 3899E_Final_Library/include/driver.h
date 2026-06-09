#ifndef DRIVER_H
#define DRIVER_H

// void toggleMatchloader();
// Main driver control loop — split-arcade drive with colour-sort and mechanism bindings.
void driverControl();

// Simple tank drive for testing and tuning; no curves or special features.
void driverControlTankTest();

void AITracking(std::pmr::string teamColor);
void opScoring(std::pmr::string teamColor);

#endif // DRIVER_H
