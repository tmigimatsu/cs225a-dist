/*
 * RedisDriver.cpp
 *
 *  Created on: May 4, 2017
 *      Author: Toki Migimatsu
 */

#include "RedisDriver.h"

#include <iostream>
#include <signal.h>

using namespace Puma;


/**********************
 * Driver Application *
 **********************/

static volatile bool g_runloop = true;

static void setCtrlCHandler(void (*userCallback)(int)) {
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = userCallback;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

static void stop(int signal) { g_runloop = false; }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

int main(int argc, char** argv)
{
	// Usage
	if (argc < 3) {
		std::cout << "Usage: puma_driver [-s REDIS_SERVER_IP] [-p REDIS_SERVER_PORT]" << std::endl
		          << std::endl
		          << "This program provides a Redis interface for communication with the Puma server." << std::endl
		          << std::endl
		          << "Optional arguments:" << std::endl
		          << "  -s REDIS_SERVER_IP" << std::endl
		          << "\t\t\t\tRedis server IP (default " << RedisServer::DEFAULT_IP << ")." << std::endl
		          << "  -p REDIS_SERVER_PORT" << std::endl
		          << "\t\t\t\tRedis server port (default " << RedisServer::DEFAULT_PORT << ")." << std::endl
		          << std::endl;
	}

	// Parse arguments
	std::string redis_ip = RedisServer::DEFAULT_IP;
	int redis_port = RedisServer::DEFAULT_PORT;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-s")) {
			// Redis server IP
			redis_ip = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-p")) {
			// Redis server port
			sscanf(argv[++i], "%d", &redis_port);
		}
	}

	// Initialize driver
	RedisDriver pumaDriver;
	pumaDriver.init();
	std::cout << "Successfully connected to Puma server." << std::endl;

	// Set up ctrl-c handler
	setCtrlCHandler(stop);

	// Run forever
	std::cout << "Running Puma driver..." << std::endl;
	pumaDriver.run();
}


/************************************
 * RedisDriver Class Implementation *
 ************************************/

void RedisDriver::eigenVectorFromBuffer(Eigen::VectorXd& vector) {
	assert(vector.size() < SIZE_OP_SPACE_TASK);
	for (int i = 0; i < vector.size(); i++) {
		vector[i] = data_buffer_[i];
	}
}

void RedisDriver::eigenVectorToBuffer(const Eigen::VectorXd& vector) {
	assert(vector.size() < SIZE_OP_SPACE_TASK);
	for (int i = 0; i < vector.size(); i++) {
		data_buffer_[i] = vector[i];
	}
}

void RedisDriver::init(const std::string& redis_hostname,
                       const int redis_port) {
	// Connect to Puma server
	puma_robot_ = std::make_unique<RobotCom>();

	// Get Puma joint angles
	puma_robot_->getStatus(GET_JPOS, data_buffer_);
	eigenVectorFromBuffer(q_);

	// Initialize timer
	timer_.setLoopFrequency(kControlFreq);
	timer_.setCtrlCHandler(&stop);
	timer_.initializeTimer(kInitializationPause);

	// Connect to Redis server
	redis_.connect(redis_hostname, redis_port);

	// Initialize Redis keys
	redis_.mset({
		{KEY_CONTROL_MODE, kDefaultControlModeStr},
		{KEY_COMMAND_DATA, RedisClient::encodeEigenMatrix(command_data_)},
		{KEY_KP, kp_str_},
		{KEY_KV, kv_str_}
	});
	redis_.setEigenMatrix(KEY_JOINT_POSITIONS, q_);
	redis_.setEigenMatrix(KEY_JOINT_VELOCITIES, dq_);
	redis_.setEigenMatrix(KEY_JOINT_TORQUES, Gamma_);
}

void RedisDriver::run() {
	while (g_runloop) {
		// Wait for timer
		timer_.waitForNextLoop();

		// Get Redis control keys in atomic transaction
		auto values = redis_.mget({KEY_CONTROL_MODE, KEY_COMMAND_DATA});

		// Check for BREAK command
		std::string mode_str = values[0];
		if (mode_str == "BREAK") {
			puma_robot_->_break();
			std::cout << "BREAK" << std::endl;
			break;
		}

		// Parse control mode
		auto mode_keyval = CONTROL_MODE_MAP.find(mode_str);
		if (mode_keyval == CONTROL_MODE_MAP.end()) {
			std::cerr << "ERROR: Redis key (" << KEY_CONTROL_MODE << ", "
			         << mode_str << ") is not a known control type." << std::endl;
			break;
		}
		control_mode_ = mode_keyval->second;

		// Filter command data
		try {
			command_data_ = RedisClient::decodeEigenMatrix(values[1]);
		} catch (std::exception& e) {
			std::cerr << e.what() << std::endl;
			break;
		}
		if (isnan(command_data_)) {
			std::cerr << "ERROR: Redis key (" << KEY_COMMAND_DATA << ", "
			         << command_data_.transpose()
			         << ") contains NaN values." << std::endl;
			break;
		}

		// Check command length
		switch (control_mode_) {
			case FLOAT:
			case NJHOLD:  case JHOLD:
			case NHOLD:   case HOLD:
				command_data_ = Eigen::VectorXd::Zero(0);
				break;
			case NJMOVE:  case JMOVE:
			case NJGOTO:  case JGOTO:
			case NJTRACK: case JTRACK:
				if (command_data_.size() != DOF) {
					std::cerr << "ERROR: Redis key (" << KEY_COMMAND_DATA << ", "
					          << command_data_.transpose() << ") must be of length "
					          << DOF << " for control mode_ " << control_mode_ << "."
					          << std::endl;
					continue;
				}
				break;
			case NXTRACK: case XTRACK:
			case NGOTO:   case GOTO:
			case NTRACK:  case TRACK:
				if (command_data_.size() != SIZE_OP_SPACE_TASK) {
					std::cerr << "ERROR: Redis key (" << KEY_COMMAND_DATA << ", "
					          << command_data_.transpose() << ") must be of length "
					          << SIZE_OP_SPACE_TASK << " for control mode_ "
					          << control_mode_ << "." << std::endl;
					continue;
				}
				break;
			default:
				break;
		}

		// Send kp if changed
		std::string kp_str_new = redis_.get(KEY_KP);
		if (kp_str_new != kp_str_) {
			try {
				kp_ = RedisClient::decodeEigenMatrix(kp_str_new);
			} catch (std::exception& e) {
				std::cerr << e.what() << std::endl;
				break;
			}
			if (kp_.size() != DOF) {
				std::cerr << "ERROR: Redis key (" << KEY_KP << ", "
			         	 << kp_.transpose() << ") must be of length " << DOF
			         	 << "." << std::endl;
				break;
			} else if (isnan(kp_)) {
				std::cerr << "ERROR: Redis key (" << KEY_KP << ", "
				          << kp_.transpose()
				          << ") contains NaN values." << std::endl;
				break;
			}
			kp_str_ = kp_str_new;
			eigenVectorToBuffer(kp_);
			puma_robot_->setStatus(SET_KP, data_buffer_, kp_.size());
		}

		// Send kv if changed
		std::string kv_str_new = redis_.get(KEY_KV);
		if (kv_str_new != kv_str_) {
			try {
				kv_ = RedisClient::decodeEigenMatrix(kv_str_new);
			} catch (std::exception& e) {
				std::cerr << e.what() << std::endl;
				break;
			}
			if (kv_.size() != DOF) {
				std::cerr << "ERROR: Redis key (" << KEY_KV << ", "
			         	 << kv_.transpose() << ") must be of length " << DOF
			         	 << "." << std::endl;
				break;
			} else if (isnan(kv_)) {
				std::cerr << "ERROR: Redis key (" << KEY_KV << ", "
				          << kv_.transpose()
				          << ") contains NaN values." << std::endl;
				break;
			}
			kv_str_ = kv_str_new;
			eigenVectorToBuffer(kv_);
			puma_robot_->setStatus(SET_KV, data_buffer_, kv_.size());
		}

		// Send command if valid
		eigenVectorToBuffer(command_data_);
		puma_robot_->control(control_mode_, data_buffer_, command_data_.size());

		// Get Puma joint positions
		puma_robot_->getStatus(GET_JPOS, data_buffer_);
		eigenVectorFromBuffer(q_);
		redis_.setEigenMatrix(KEY_JOINT_POSITIONS, q_);

		// Get Puma joint velocities
		puma_robot_->getStatus(GET_JVEL, data_buffer_);
		eigenVectorFromBuffer(dq_);
		redis_.setEigenMatrix(KEY_JOINT_VELOCITIES, dq_);

		// Get Puma torques
		puma_robot_->getStatus(GET_TORQ, data_buffer_);
		eigenVectorFromBuffer(Gamma_);
		redis_.setEigenMatrix(KEY_JOINT_TORQUES, Gamma_);
	}

	// Break robot
	puma_robot_->_break();
	std::cout << "Sent BREAK command to Puma. Stopping run loop." << std::endl;
}

