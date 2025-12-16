#ifndef AUTON_H
#define AUTON_H

//void testNormalizeAngle();
//void driveForDistance(double distanceCm, double heading); 
//void driveForDistancePID(double distanceCm, double heading); 

void autonLeft();
void autonRight();
void autonTest();
void autonFwdRight();
void autonFwdLeft();
void SpeedwayAutonLeft();
void SevenBallRight();
void odomTest();
void calibration();
void soloAWP();
void SevenBallLeft();


// Distance sensor detection function
bool detectDistanceDecline(vex::distance& sensor, double declineThreshold);

// Cup pickup routine
void cupPickupAndDeliver();

#endif // AUTON_H