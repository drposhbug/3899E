// ============================================================================
// NAVIGATION.CPP - Motion Control Implementation
// ============================================================================
#include "navigation.h"
#include "robot_config.h"
#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace vex;

// ======================== BASIC MOVEMENT FUNCTIONS ==========================

/**
 * Move robot forward/backward for a specified distance
 * @param distanceCM Distance to travel in centimeters
 * @param maxSpeed Maximum speed percentage (0-100)
 * @param dir Direction (forward or reverse)
 */
void move(double distanceCM, double maxSpeed, vex::directionType dir = vex::forward);

/**
 * Arc turn with differential wheel speeds
 * @param targetDistance Arc length to travel
 * @param breakDistance Distance from target to begin deceleration
 * @param minSpeed Minimum speed during approach phase
 * @param maxSpeed Maximum speed during movement
 * @param turnRadius Radius of the arc in centimeters
 * @param turnLeft True for left arc, false for right arc
 */
void arcTurn(double targetDistance, 
             double breakDistance,
             double minSpeed,
             double maxSpeed,
             double turnRadius,
             bool turnLeft);

// ======================== ADVANCED MOVEMENT FUNCTIONS =======================

/**
 * Drive straight with motion profiling and heading correction
 * Features: Traction control, ABS braking, PID heading correction
 * @param targetDistance Distance to travel in cm (negative for backward)
 * @param breakDistance Distance from target to begin deceleration
 * @param targetHeading Desired robot heading in degrees (0-360)
 * @param minSpeed Minimum speed during approach phase
 * @param kp_heading PID proportional gain for heading correction
 * @param ki_heading PID integral gain for heading correction
 * @param kd_heading PID derivative gain for heading correction
 * @param accelHeadingScaling Heading correction strength during acceleration
 * @param decelHeadingScaling Heading correction strength during deceleration
 * @param approachHeadingScaling Heading correction strength during final approach
 * @param maxSpeed Maximum speed percentage
 */
void straight(double targetDistance, 
              double breakDistance, 
              double targetHeading = 0, 
              double minSpeed = 16, 
              double kp_heading = 0.4, 
              double ki_heading = 0.01, 
              double kd_heading = 0.05, 
              double accelHeadingScaling = 0.2, 
              double decelHeadingScaling = 0.2, 
              double approachHeadingScaling = 0.2, 
              double maxSpeed = 100);

/**
 * Rotate robot to target heading with motion profiling
 * Features: Traction control, ABS braking, synchronized wheel control
 * @param targetHeading Target heading in degrees
 * @param breakDistanceInDegrees Degrees from target to begin deceleration
 * @param minSpeed Minimum rotational speed
 * @param maxSpeed Maximum rotational speed
 */
void turn(double targetHeading, 
          double breakDistanceInDegrees, 
          double minSpeed = 17, 
          double maxSpeed = 100);

/**
 * Pivot turn - one side stationary, other side rotates
 * More precise but slower than standard turn
 * @param targetHeading Target heading in degrees
 * @param breakDistanceInDegrees Degrees from target to begin deceleration
 * @param minSpeed Minimum rotational speed
 * @param maxSpeed Maximum rotational speed
 */
void pivotTurn(double targetHeading,
               double breakDistanceInDegrees,
               double minSpeed = 17,
               double maxSpeed = 100);

// ======================== CONVENIENCE WRAPPER FUNCTIONS =====================

/**
 * Move forward with motion profiling
 * Wrapper for straight() with positive distance
 */
void forwardMP(double targetDistance,
               double breakDistance = 35, 
               double targetHeading = 0,
               double minSpeed = 16,
               double kp_heading = 1, 
               double ki_heading = 0.01,
               double kd_heading = 0.05, 
               double accelHeadingScaling = 0.20,
               double decelHeadingScaling = 0.20, 
               double approachHeadingScaling = 0.20,
               double maxSpeed = 100);

/**
 * Move backward with motion profiling
 * Wrapper for straight() with negative distance
 */
void backwardMP(double targetDistance,
                double breakDistance = 35, 
                double targetHeading = 0,
                double minSpeed = 17,
                double kp_heading = 0.4, 
                double ki_heading = 0.01,
                double kd_heading = 0.1, 
                double accelHeadingScaling = 0.20,
                double decelHeadingScaling = 0.20, 
                double approachHeadingScaling = 0.20,
                double maxSpeed = 100);

/**
 * Turn left (counter-clockwise) by specified degrees
 * @param turnAmount Degrees to turn (positive)
 */
void leftMP(double turnAmount, 
            double breakDistance = 35, 
            double minSpeed = 17, 
            double maxSpeed = 100);

/**
 * Turn right (clockwise) by specified degrees
 * @param turnAmount Degrees to turn (positive)
 */
void rightMP(double turnAmount,
             double breakDistance = 35,
             double minSpeed = 17,
             double maxSpeed = 100);

/**
 * Pivot turn left by specified degrees
 * @param turnAmount Degrees to pivot (positive)
 */
void pivotLeftMP(double turnAmount, 
                 double breakDistance, 
                 double minSpeed, 
                 double maxSpeed);

/**
 * Pivot turn right by specified degrees
 * @param turnAmount Degrees to pivot (positive)
 */
void pivotRightMP(double turnAmount, 
                  double breakDistance, 
                  double minSpeed, 
                  double maxSpeed);

#endif // NAVIGATION_H