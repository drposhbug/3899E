#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"

/**
 * Project Specific Headers
 * Including them here makes them available anywhere main.h is included
 */
#include "robot_config.hpp"
#include "utils.hpp"
#include "driver.hpp"
#include "auton.hpp"
#include "navigation.hpp"
#include "autontasks.hpp"

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_