/*
 * RedisDriver.h
 *
 *  Created on: May 8, 2017
 *      Author: Toki Migimatsu
 */

#ifndef PUMA_REDIS_DRIVER_H
#define PUMA_REDIS_DRIVER_H

#include "RobotCom.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <string>
#include <thread>
#include <map>

#include <Eigen/Core>

namespace Puma {

/*************
 * Constants *
 *************/

// Puma degrees of freedom
const int DOF = 6;

// Operational space command size
// Format:
//     3 parameters for position (XYZ Cartesian)
//     4 parameters for orientation (WXYZ Euler parameters)
//     [pos_x pos_y pos_z quat_w quat_x quat_y quat_z]
const int SIZE_OP_SPACE_TASK = 7;

// Redis keys sent to robot
const std::string KEY_CONTROL_MODE = "cs225a::robot::puma::tasks::control_mode";
const std::string KEY_COMMAND_DATA = "cs225a::robot::puma::tasks::command_data";
const std::string KEY_KP           = "cs225a::robot::puma::tasks::kp";
const std::string KEY_KV           = "cs225a::robot::puma::tasks::kv";
const std::string KEY_VMAX         = "cs225a::robot::puma::tasks::vmax";

// Redis keys returned by robot
const std::string KEY_JOINT_POSITIONS  = "cs225a::robot::puma::sensors::q";
const std::string KEY_JOINT_VELOCITIES = "cs225a::robot::puma::sensors::dq";
const std::string KEY_JOINT_TORQUES    = "cs225a::robot::puma::actuators::fgc";
const std::string KEY_EE_POS           = "cs225a::robot::puma::tasks::ee_pos";
const std::string KEY_EE_ORI           = "cs225a::robot::puma::tasks::ee_ori";

// Puma control modes
const std::map<std::string, ControlMode> CONTROL_MODE_MAP = {

	// Default mode:
	{"FLOAT", FLOAT},    // Float with gravity compensation (0 params)

	// Hold configuration:
	{"HOLD", HOLD},      // Operational space (0 params)
	{"JHOLD", JHOLD},    // Joint space (0 params)
	{"NHOLD", NHOLD},    // Operational space without dynamic decoupling (0 params)
	{"NJHOLD", NJHOLD},  // Joint space without dynamic decoupling (0 params)

	// Move to configuration:
	{"JMOVE", JMOVE},    // Joint space (6 params)
	{"NJMOVE", NJMOVE},  // Joint space without dynamic decoupling (6 params)

	// Move to configuration with velocity saturation:
	{"GOTO", GOTO},      // Operational space (7 params)
	{"JGOTO", JGOTO},    // Joint space (6 params)
	{"NGOTO", NGOTO},    // Operational space without dynamic decoupling (7 params)
	{"NJGOTO", NJGOTO},  // Joint space without dynamic decoupling (6 params)

	// Move to configuration with cubic splines:
	{"TRACK", TRACK},      // Operational space (7 params)
	{"XTRACK", XTRACK},    // Inverse kinematics (7 params)
	{"JTRACK", JTRACK},    // Joint space (6 params)
	{"NTRACK", NTRACK},    // Operational space without dynamic decoupling (7 params)
	{"NXTRACK", NXTRACK},  // Inverse kinematics without dynamic decoupling (7 params)
	{"NJTRACK", NJTRACK},  // Joint space without dynamic decoupling (6 params)

	// Other controllers:
	// TODO: Redis interface not implemented
	{"PFMOVE", PFMOVE},    // GOTO control with obstacle avoidance
	{"LINE", LINE},        // Move along line
	{"PROJ1", PROJ1}, {"PROJ2", PROJ2}, {"PROJ3", PROJ3}  // Custom controllers

};


/********************************
 * RedisDriver Class Definition *
 ********************************/

class RedisDriver {

public:

	RedisDriver() :
		command_data_(DOF),
		q_(DOF),
		dq_(DOF),
		Gamma_(DOF),
		x_(SIZE_OP_SPACE_TASK),
		kp_(DOF),
		kv_(DOF)
	{
		command_data_.setZero();
		q_.setZero();
		dq_.setZero();
		Gamma_.setZero();
		x_.setZero();
		kp_.fill(kDefaultKp);
		kv_.fill(kDefaultKv);
		kp_str_ = RedisClient::encodeEigenMatrixString(kp_);
		kv_str_ = RedisClient::encodeEigenMatrixString(kv_);
	}

	// Constants
	const int kControlFreq = 1000;         // 1kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop
	const int kDefaultKp = 400;
	const int kDefaultKv = 40;
	const std::string kDefaultControlModeStr = "FLOAT";
	const ControlMode kDefaultControlMode = CONTROL_MODE_MAP.at(kDefaultControlModeStr);

	const HiredisServerInfo kRedisServerInfo = {
		"127.0.0.1",  // hostname
		6379,         // port
		{ 1, 500000 } // timeout = 1.5 seconds
	};

	// Connect to Puma server and initialize Redis client/timer
	void init();

	// Run driver in a loop
	void run();

	// The following need to be public so that the ctrl-c handler can call
	// puma_robot_->_break() and set runloop_ = false.
	std::unique_ptr<RobotCom> puma_robot_;
	volatile bool runloop_ = true;

protected:

	// Convert between data_buffer_ and Eigen::VectorXd
	void eigenVectorFromBuffer(Eigen::VectorXd& vector);
	void eigenVectorToBuffer(const Eigen::VectorXd& vector);

	ControlMode parseControlMode(std::string& control_mode_str) const;
	Eigen::VectorXd parseCommandData(std::string& command_data_str, ControlMode control_mode) const;
	Eigen::VectorXd parseKp(std::string& kp_str) const;
	Eigen::VectorXd parseKv(std::string& kv_str) const;

	// Redis client and timer
	RedisClient redis_client_;
	LoopTimer timer_;

	// Controller variables
	ControlMode control_mode_ = kDefaultControlMode;
	Eigen::VectorXd command_data_;
	Eigen::VectorXd q_;
	Eigen::VectorXd dq_;
	Eigen::VectorXd Gamma_;
	Eigen::VectorXd x_;
	Eigen::VectorXd kp_;
	Eigen::VectorXd kv_;
	float vmax_ = 0;
	std::string control_mode_str_;
	std::string command_data_str_;
	std::string kp_str_;
	std::string kv_str_;
	std::string vmax_str_;

	// Buffer of data for communication with Puma
	float data_buffer_[SIZE_OP_SPACE_TASK] = {0};

};

}

#endif  // PUMA_REDIS_DRIVER_H
