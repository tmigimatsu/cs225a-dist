#ifndef SAI2_KINOVA_JACO_H
#define SAI2_KINOVA_JACO_H

#include <string>
#include <Eigen/Core>

#include "redis/RedisClient.h"

namespace KinovaJaco {

const std::string KEY_PREFIX = RedisServer::KEY_PREFIX + "kinova_jaco::";
// Redis keys sent to robot
const std::string KEY_COMMAND_JOINT_POSITIONS = KEY_PREFIX + "actuators::q_des";
const std::string KEY_COMMAND_TORQUES         = KEY_PREFIX + "actuators::fgc";
const std::string KEY_CONTROL_MODE            = KEY_PREFIX + "control_mode";

// Redis keys returned by robot
const std::string KEY_JOINT_POSITIONS         = KEY_PREFIX + "sensors::q";
const std::string KEY_JOINT_VELOCITIES        = KEY_PREFIX + "sensors::dq";

// Gravity file
const std::string GRAVITY_FILENAME = "resources/kinova_jaco_driver/gravity.xml";

// Kinova Jaco degrees of freedom
const int DOF = 6;

// Kinova driver control modes
enum ControlMode {
	READ_POSITIONS,
	COMMAND_POSITIONS,
	COMMAND_TORQUES,
	COMMAND_FINGERS
};

const std::string CONTROL_MODES[] = {
	"READ_POSITIONS",
	"COMMAND_POSITIONS",
	"COMMAND_TORQUES",
	"COMMAND_FINGERS"
};

enum FingerControl {
	OFF,
	OPEN,
	CLOSE,
	OPEN_CLOSE
};

// Kinova home position (approximate)
const double _ARR_HOME_POSITION[DOF] = {-0.0963601, -0.2165757, 2.13941058, 0.50438513, 1.69296152, -6.0282042};
const Eigen::VectorXd HOME_POSITION = Eigen::Matrix<double,DOF,1>(_ARR_HOME_POSITION);

// Torque limits
// TODO: Find real limits
const double _ARR_TORQUE_LIMITS[DOF] = {10, 10, 10, 10, 10, 10};
const Eigen::ArrayXd TORQUE_LIMITS = Eigen::Array<double,DOF,1>(_ARR_TORQUE_LIMITS);

// SAI to Kinova angle offset (in degrees)
const double _ARR_KINOVA_ANGLE_OFFSET[DOF] = {270.0, 180.0, 180.0, 270.0, 180.0, 90.0};
const Eigen::ArrayXd KINOVA_ANGLE_OFFSET = Eigen::Array<double,DOF,1>(_ARR_KINOVA_ANGLE_OFFSET);

// SAI to Kinova angle sign
const double _ARR_KINOVA_ANGLE_SIGN[DOF] = {-1, 1, -1, -1, -1, -1};
const Eigen::ArrayXd KINOVA_ANGLE_SIGN = Eigen::Array<double,DOF,1>(_ARR_KINOVA_ANGLE_SIGN);

}  // namespace KinovaJaco

#endif  // SAI2_KINOVA_JACO_H
