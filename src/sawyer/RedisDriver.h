/*
 * RedisDriver.h
 *
 *  Created on: July 14, 2017
 *      Author: Toki Migimatsu
 */

#ifndef SAWYER_REDIS_DRIVER_H
#define SAWYER_REDIS_DRIVER_H

#include <string>

namespace Sawyer {

/*************
 * Constants *
 *************/

// Sawyer degrees of freedom
const int DOF = 7;

const std::string KEY_PREFIX = "cs225a::sawyer::";
// Redis keys sent to robot
const std::string KEY_COMMAND_TORQUES   = KEY_PREFIX + "actuators::fgc";
const std::string KEY_COMMAND_POSITIONS = KEY_PREFIX + "sensors::q";

// Redis keys returned by robot
const std::string KEY_JOINT_POSITIONS  = KEY_PREFIX + "sensors::q";
const std::string KEY_JOINT_VELOCITIES = KEY_PREFIX + "sensors::dq";

}

#endif  // SAWYER_REDIS_DRIVER_H
