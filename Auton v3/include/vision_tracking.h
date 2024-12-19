#include "vex.h"

#ifndef VISION_TRACKING_H
#define VISION_TRACKING_H

#include "vex.h"

// Function to follow an object using AI vision
void followObject(vex::aivision &visionSensor, double maxVelocity, double distanceThreshold = 10.0);

#endif // VISION_TRACKING_H