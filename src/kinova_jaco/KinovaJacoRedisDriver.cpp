/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright Jul 2016, Stanford Robotics Lab, Stanford University
 *
 * Author : Toki Migimatsu
 * Author : Vikranth Reddy
 * Author : Vinay Sriram
 * Author : Samir Menon
 */

#include "KinovaJacoRedisDriver.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <math.h>

#include <tinyxml2.h>

using namespace KinovaJaco;

static const float RAD_PER_DEG = M_PI / 180.0;
static const float DEG_PER_RAD = 180.0 / M_PI;

static volatile bool g_runloop = true;

// Ctrl-C handler
void stop(int) {
	if (!g_runloop) exit(1);
	g_runloop = false;
}

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

// Convert Eigen vector to Kinova joint positions
static AngularInfo eigenVectorToKinovaAngular(const Eigen::VectorXd& eigen_vector) {
	if (eigen_vector.size() != KinovaJaco::DOF)
		throw std::runtime_error("eigenToKinovaPosition(): ERROR : Eigen::VectorXd must contain " + std::to_string(KinovaJaco::DOF) + " elements.");

	AngularInfo kinova_angular = {static_cast<float>(eigen_vector[0]),
	                              static_cast<float>(eigen_vector[1]),
	                              static_cast<float>(eigen_vector[2]),
	                              static_cast<float>(eigen_vector[3]),
	                              static_cast<float>(eigen_vector[4]),
	                              static_cast<float>(eigen_vector[5])};
	return kinova_angular;
}

// Convert Kinova joint positions to Eigen vector
static Eigen::VectorXd kinovaAngularToEigenVector(const AngularInfo& kinova_angular) {
	Eigen::VectorXd eigen_vec(KinovaJaco::DOF);
	eigen_vec << kinova_angular.Actuator1, kinova_angular.Actuator2, kinova_angular.Actuator3,
	             kinova_angular.Actuator4, kinova_angular.Actuator5, kinova_angular.Actuator6;
	return eigen_vec;
}

int main(int argc, char *argv[]) {
	signal(SIGINT, stop);

	// Usage
	std::cout << "Usage: kinova_jaco_driver [-rs REDIS_SERVER_IP] [-rp REDIS_SERVER_PORT]" << std::endl
		      << "                          [-g GRAVITY_XML]" << std::endl
	          << std::endl
	          << "This driver provides a Redis interface for communication with the Kuka IIWA." << std::endl
	          << std::endl
	          << "Specify the control mode via Redis key: " << KEY_CONTROL_MODE << std::endl
	          << "  0: " << CONTROL_MODES[0]
	          << "\t1: " << CONTROL_MODES[1]
	          << "\t2: " << CONTROL_MODES[2]
	          << "\t3: " << CONTROL_MODES[3] << std::endl
	          << std::endl
	          << "Optional arguments:" << std::endl
	          << "  -rs REDIS_SERVER_IP" << std::endl
	          << "\t\t\t\tRedis server IP (default " << RedisServer::DEFAULT_IP << ")." << std::endl
	          << "  -rp REDIS_SERVER_PORT" << std::endl
	          << "\t\t\t\tRedis server port (default " << RedisServer::DEFAULT_PORT << ")." << std::endl
	          << "  -g GRAVITY_XML" << std::endl
	          << "\t\t\t\tKinova gravity file (default " << KinovaJaco::GRAVITY_FILENAME << ")." << std::endl
	          << std::endl;

	// Parse arguments
	std::string redis_ip         = RedisServer::DEFAULT_IP;
	int redis_port               = RedisServer::DEFAULT_PORT;
	std::string gravity_filename = KinovaJaco::GRAVITY_FILENAME;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-rs")) {
			// Redis server IP
			redis_ip = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-rp")) {
			// Redis server port
			sscanf(argv[++i], "%d", &redis_port);
		} else if (!strcmp(argv[i], "-g")) {
			// Gravity XML
			gravity_filename = std::string(argv[++i]);
		}
	}

	try {
		KinovaJacoRedisDriver app(redis_ip, redis_port, gravity_filename);
		app.connect();
		app.run();
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		return 1;
	}

	return 0;
}


KinovaJacoRedisDriver::KinovaJacoRedisDriver(const std::string& redis_ip, const int redis_port,
                                             const std::string& gravity_filename) {
	// Initialize Redis
	redis_.connect(redis_ip, redis_port);
	redis_.setEigenMatrix(KEY_COMMAND_TORQUES, Eigen::VectorXd::Zero(DOF));
	redis_.setEigenMatrix(KEY_COMMAND_JOINT_POSITIONS, Eigen::VectorXd::Zero(DOF));
	redis_.setEigenMatrix(KEY_JOINT_POSITIONS, Eigen::VectorXd::Zero(DOF));
	redis_.set(KEY_CONTROL_MODE, std::to_string(0));

	// Initialize timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	// Initialize variables
	parseGravity(gravity_filename);
	command_joint_positions_.InitStruct();
	command_joint_positions_.Position.Type = ANGULAR_POSITION;
}

void KinovaJacoRedisDriver::connect() {
	// Initialize API
	if (InitAPI() != 1)
		throw std::runtime_error("ERROR : Could not initialize API. Make sure robot is connected." );

	// Test communication
	AngularPosition kinova_position;
 	if (GetAngularCommand(kinova_position) != 1) {
		CloseAPI();
		throw std::runtime_error("ERROR : Communication test with robot failed." );
	}
}

void KinovaJacoRedisDriver::run() {
	// Initialize robot
	std::cout << "Moving to home position..." << std::endl;
	SwitchTrajectoryTorque(POSITION);
	MoveHome();
	InitFingers();

	// Run loop
	std::cout << "Waiting for commands. Control mode set to: " << CONTROL_MODES[control_mode_] << std::endl;
	while (g_runloop) {
		timer_.waitForNextLoop();
		try {
			readRedisValues();
			writeRedisValues();
			sendRobotCommand();
		} catch (std::exception& e) {
			std::cout << e.what() << std::endl
			          << "Exiting." << std::endl;
			break;
		}
	}

	// Reset robot
	if (control_mode_ == COMMAND_TORQUES) {
		command_torques_ = Eigen::VectorXf::Zero(DOF);
		SendAngularTorqueCommand(arr_command_torques_);
	}

	// Close the API
	CloseAPI();
}

void KinovaJacoRedisDriver::parseGravity(const std::string& gravity_filename) {
	tinyxml2::XMLDocument doc;
	doc.LoadFile(gravity_filename.c_str());
	if (doc.Error()) {
		std::cout << "WARNING : Could not load gravity file: " << gravity_filename << std::endl;
		return;
	}

	std::cout << "Loading gravity file: " << gravity_filename << std::endl;
	try {
		const char *gravity = doc.FirstChildElement("gravity")->GetText();
		std::cout << "Gravity: " << gravity << std::endl << std::endl;
		std::stringstream ss(gravity);
		for (int i = 0; i < OPTIMAL_Z_PARAM_SIZE; i++) {
			ss >> kGravity[i];
		}
	} catch (const std::exception& e) {
		std::cout << e.what();
		throw std::runtime_error("ERROR : Failed to parse gravity file.");
	}
}

void KinovaJacoRedisDriver::readRedisValues() {
	ControlMode new_control_mode = static_cast<ControlMode>(std::stoi(redis_.get(KEY_CONTROL_MODE)));

	if (new_control_mode != control_mode_) {
		control_mode_ = new_control_mode;
		std::cout << "Switching control mode to: " << CONTROL_MODES[control_mode_] << std::endl;

		// Initialize control mode
		if (control_mode_ == COMMAND_TORQUES) {
			// Set gravity
			SetGravityOptimalZParam(kGravity);
			SetGravityType(OPTIMAL);

			// Zero out the torque commands
			command_torques_ = Eigen::VectorXf::Zero(DOF);

			// Set the torque control type to Direct Torque Control
			SwitchTrajectoryTorque(TORQUE);
			SetTorqueControlType(DIRECTTORQUE);

			// Set safety settings on
			SetTorqueSafetyFactor(1);
			SetTorqueVibrationController(1);

			// Initialize the torque commands
			SendAngularTorqueCommand(arr_command_torques_);
		}
	}

	// Read command from Redis
	switch (control_mode_) {
		case COMMAND_POSITIONS: {
			// Get command joint positions
			Eigen::VectorXd q_des = redis_.getEigenMatrix(KEY_COMMAND_JOINT_POSITIONS);
			q_des = KINOVA_ANGLE_SIGN * q_des.array() * DEG_PER_RAD + KINOVA_ANGLE_OFFSET;
			command_joint_positions_.Position.Actuators = eigenVectorToKinovaAngular(q_des);
			break;
		}

		case COMMAND_TORQUES: {
			// Get command torques
			Eigen::VectorXd new_command_torques = redis_.getEigenMatrix(KEY_COMMAND_TORQUES);
			new_command_torques = KINOVA_ANGLE_SIGN * new_command_torques.array();
			command_torques_ = new_command_torques.cast<float>();
			break;
		}

		default:
			break;
	}
}

void KinovaJacoRedisDriver::writeRedisValues() {
	// Convert joint positions
	AngularPosition kinova_position;
	GetAngularPosition(kinova_position);
	Eigen::VectorXd q = kinovaAngularToEigenVector(kinova_position.Actuators);
	q = KINOVA_ANGLE_SIGN * (q.array() - KINOVA_ANGLE_OFFSET) * RAD_PER_DEG;

	// Convert joint velocities
	AngularPosition kinova_velocity;
	GetAngularVelocity(kinova_velocity);
	Eigen::VectorXd dq = kinovaAngularToEigenVector(kinova_velocity.Actuators);
	dq = KINOVA_ANGLE_SIGN * dq.array() * RAD_PER_DEG;

	// Write redis values
	redis_.pipeset({{KEY_JOINT_POSITIONS,  RedisClient::encodeEigenMatrix(q)},
	                {KEY_JOINT_VELOCITIES, RedisClient::encodeEigenMatrix(dq)}});
}

void KinovaJacoRedisDriver::sendRobotCommand() {
	switch (control_mode_) {

		case COMMAND_POSITIONS:
			SendAdvanceTrajectory(command_joint_positions_);
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			break;

		case COMMAND_TORQUES:
			if (isnan(command_torques_)) {
				std::stringstream ss;
				ss << "ERROR : Command torques [" << command_torques_.transpose() << "] contain NaN values." << std::endl;
				throw std::runtime_error(ss.str());
			}
			if ((command_torques_.array().abs() > TORQUE_LIMITS.cast<float>()).any()) {
				std::cout << "WARNING : Command torques [" << command_torques_.transpose() << "]" << std::endl
				  	  	  << "          exceed limits   [" << TORQUE_LIMITS.transpose() << "]." << std::endl;
				command_torques_ = command_torques_.array().min(TORQUE_LIMITS.cast<float>()).max(-TORQUE_LIMITS.cast<float>());
			}

			// Get desired joint torques from Redis
			SendAngularTorqueCommand(arr_command_torques_);
			break;

		default:
			break;
	}
}

