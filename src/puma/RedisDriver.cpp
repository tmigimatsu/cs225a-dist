/*
 * RedisDriver.cpp
 *
 *  Created on: May 4, 2017
 *      Author: Toki Migimatsu
 */

#include "RedisDriver.h"

#include <iostream>
#include <stdexcept>
#include <signal.h>

using namespace Puma;

/**********************
 * Driver Application *
 **********************/

static volatile bool g_runloop = true;
static RobotCom *g_puma_robot = nullptr;

static void setCtrlCHandler(void (*userCallback)(int)) {
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = userCallback;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

static void stop() {
	g_runloop = false;
#if CONNECT_SERVER
	if (g_puma_robot != nullptr)
		g_puma_robot->_break();
#endif  // CONNECT_SERVER
	std::cout << "Sent BREAK command to Puma. Stopping run loop." << std::endl;
	exit(0);
}
static void stop(int signal) { stop(); }

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

	// Set up ctrl-c handler
#ifdef CONNECT_SERVER
	g_puma_robot = pumaDriver.puma_robot_.get();
#endif  // CONNECT_SERVER
	setCtrlCHandler(stop);

	// Run forever
	pumaDriver.run();
}


/************************************
 * RedisDriver Class Implementation *
 ************************************/

void RedisDriver::eigenVectorFromBuffer(Eigen::VectorXd& vector) {
	assert(vector.size() <= SIZE_OP_SPACE_TASK);
	for (int i = 0; i < vector.size(); i++) {
		vector(i) = data_buffer_[i];
	}
}

void RedisDriver::eigenVectorToBuffer(const Eigen::VectorXd& vector) {
	assert(vector.size() <= SIZE_OP_SPACE_TASK);
	for (int i = 0; i < vector.size(); i++) {
		data_buffer_[i] = vector(i);
	}
}

void RedisDriver::init(const std::string& redis_hostname,
                       const int redis_port) {
#if CONNECT_SERVER
	// Connect to Puma server
	puma_robot_ = std::make_unique<RobotCom>();

	// Get Puma configuration
	puma_robot_->getStatus(GET_JPOS, data_buffer_);
	eigenVectorFromBuffer(q_);
	puma_robot_->getStatus(GET_JVEL, data_buffer_);
	eigenVectorFromBuffer(dq_);
	puma_robot_->getStatus(GET_TORQ, data_buffer_);
	eigenVectorFromBuffer(Gamma_);
	std::cout << "Successfully connected to Puma server." << std::endl;
#endif  // CONNECT_SERVER

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
	redis_.set(KEY_VMAX, std::to_string(vmax_));
}

ControlMode RedisDriver::parseControlMode(std::string& control_mode_str) const {
	// Check for BREAK command
	if (control_mode_str == "BREAK") {
		puma_robot_->_break();
		throw std::runtime_error("Received BREAK: Exiting driver.");
	}

	// Map string to ControlMode
	auto mode_keyval = CONTROL_MODE_MAP.find(control_mode_str);
	if (mode_keyval == CONTROL_MODE_MAP.end()) {
		std::stringstream ss;
		ss << "ERROR: Redis key (" << KEY_CONTROL_MODE << " = "
		   << control_mode_str << ") is not a known control type." << std::endl;
		throw std::invalid_argument(ss.str());
	}
	return mode_keyval->second;
}

Eigen::VectorXd RedisDriver::parseCommandData(std::string& command_data_str, ControlMode control_mode) const {
	Eigen::VectorXd command_data = RedisClient::decodeEigenMatrixString(command_data_str);
	// Check number validity
	if (isnan(command_data)) {
		std::stringstream ss;
		ss << "ERROR: Redis key (" << KEY_COMMAND_DATA << " = "
		   << command_data.transpose() << ") contains NaN values." << std::endl;
		throw std::invalid_argument(ss.str());
	}

	// Check command length
	switch (control_mode) {
		case FLOAT:
		case NJHOLD:  case JHOLD:
		case NHOLD:   case HOLD:
			command_data = Eigen::VectorXd::Zero(0);
			break;
		case NJMOVE:  case JMOVE:
		case NJGOTO:  case JGOTO:
		case NJTRACK: case JTRACK:
			if (command_data.size() != DOF) {
				std::stringstream ss;
				ss << "ERROR: Redis key (" << KEY_COMMAND_DATA << " = "
				   << command_data.transpose() << ") must be of length " << DOF
				   << " for control mode " << control_mode << "." << std::endl;
				throw std::invalid_argument(ss.str());
			}
			// Convert radians to degrees
			command_data *= 180.0 / M_PI;
			break;
		case NXTRACK: case XTRACK:
		case NGOTO:   case GOTO:
		case NTRACK:  case TRACK:
			if (command_data.size() != SIZE_OP_SPACE_TASK) {
				// Search for string associated with ControlMode
				// (inefficient but the program is going to exit anyways)
				std::string control_mode_str;
				for (auto& mode_keyval : CONTROL_MODE_MAP) {
					if (mode_keyval.second == control_mode) {
						control_mode_str = mode_keyval.first;
						break;
					}
				}
				std::stringstream ss;
				ss << "ERROR: Redis key (" << KEY_COMMAND_DATA << " = "
				   << command_data.transpose() << ") must be of length "
				   << SIZE_OP_SPACE_TASK << " for control mode "
				   << control_mode_str << "." << std::endl;
				throw std::invalid_argument(ss.str());
			}
			break;
		default:
			break;
	}

	return command_data;
}

Eigen::VectorXd RedisDriver::parseKp(std::string& kp_str) const {
	Eigen::VectorXd kp = RedisClient::decodeEigenMatrixString(kp_str);
	if (kp.size() != DOF) {
		std::stringstream ss;
		ss << "ERROR: Redis key (" << KEY_KP << " = " << kp.transpose()
		   << ") must be of length " << DOF << "." << std::endl;
		throw(std::invalid_argument(ss.str()));
	} else if (isnan(kp)) {
		std::stringstream ss;
		ss << "ERROR: Redis key (" << KEY_KP << " = " << kp.transpose()
		   << ") contains NaN values." << std::endl;
		throw(std::invalid_argument(ss.str()));
	}
	return kp;
}

Eigen::VectorXd RedisDriver::parseKv(std::string& kv_str) const {
	Eigen::VectorXd kv = RedisClient::decodeEigenMatrixString(kv_str);
	if (kv.size() != DOF) {
		std::stringstream ss;
		ss << "ERROR: Redis key (" << KEY_KV << " = " << kv.transpose()
		   << ") must be of length " << DOF << "." << std::endl;
		throw(std::invalid_argument(ss.str()));
	} else if (isnan(kv)) {
		std::stringstream ss;
		ss << "ERROR: Redis key (" << KEY_KV << " = " << kv.transpose()
		   << ") contains NaN values." << std::endl;
		throw(std::invalid_argument(ss.str()));
	}
	return kv;
}

void RedisDriver::run() {
	std::cout << "Listening for commands..." << std::endl;
	try {
		while (g_runloop) {
			// Wait for timer
			timer_.waitForNextLoop();

			// Get Redis control keys in atomic transaction
			auto values = redis_.mget({KEY_CONTROL_MODE, KEY_COMMAND_DATA, KEY_KP, KEY_KV});

			// Check which keys have changed
			bool control_mode_changed = (control_mode_str_ != values[0]);
			bool command_data_changed = (command_data_str_ != values[1]);
			bool kp_changed = (kp_str_ != values[2]);
			bool kv_changed = (kv_str_ != values[3]);

			// Parse all new keys and check their validity
			ControlMode control_mode_new;
			Eigen::VectorXd command_data_new;
			Eigen::VectorXd kp_new;
			Eigen::VectorXd kv_new;
			if (control_mode_changed) {
				control_mode_new = parseControlMode(values[0]);
			}
			if (command_data_changed || control_mode_changed) {
				command_data_new = parseCommandData(values[1], control_mode_new);
			}
			if (kp_changed) {
				kp_new = parseKp(values[2]);
			}
			if (kv_changed) {
				kv_new = parseKv(values[3]);
			}

			// All keys are valid. Update member variables and decide which keys to send.
			bool send_control = false;
			bool send_kp = false;
			bool send_kv = false;
			if (control_mode_changed) {
				control_mode_str_ = values[0];
				control_mode_ = control_mode_new;
				send_control = true;
				send_kp = (control_mode_ != FLOAT);
				send_kv = (control_mode_ != FLOAT);
			}
			if (command_data_changed) {
				command_data_str_ = values[1];
				command_data_ = command_data_new;
				send_control = true;
			}
			if (kp_changed) {
				kp_str_ = values[2];
				kp_ = kp_new;
				send_kp = true;
			}
			if (kv_changed) {
				kv_str_ = values[3];
				kv_ = kv_new;
				send_kv = true;
			}

			// Send commands to Puma
			if (send_kp) {
#if CONNECT_SERVER
				eigenVectorToBuffer(kp_);
				puma_robot_->setStatus(SET_KP, control_mode_, data_buffer_, kp_.size());
#endif  // CONNECT_SERVER
				std::cout << "Update KP: " << control_mode_str_ << " " << kp_str_ << std::endl;
			}

			if (send_kv) {
#if CONNECT_SERVER
				eigenVectorToBuffer(kv_);
				puma_robot_->setStatus(SET_KV, control_mode_, data_buffer_, kv_.size());
#endif  // CONNECT_SERVER
				std::cout << "Update KV: " << control_mode_str_ << " " << kv_str_ << std::endl;
			}

			if (send_control) {
#if CONNECT_SERVER
				eigenVectorToBuffer(command_data_);
				puma_robot_->control(control_mode_, data_buffer_, command_data_.size());
#endif  // CONNECT_SERVER
				std::cout << "Update Control: " << control_mode_str_ << " "
				          << (control_mode_ == FLOAT ? "" : command_data_str_) << std::endl;
			}

			// Set VMAX
			std::string vmax_str_new = redis_.get(KEY_VMAX);
			if (vmax_str_new != vmax_str_) {
				vmax_ = std::stof(vmax_str_new);
				if (vmax_ != 0) {
#if CONNECT_SERVER
					puma_robot_->setConstant(SET_VMAX, vmax_);
#endif  // CONNECT_SERVER
					std::cout << "Update VMAX: " << vmax_ << std::endl;
				}
				vmax_str_ = vmax_str_new;
			}

#if CONNECT_SERVER
			// Get Puma joint positions
			puma_robot_->getStatus(GET_JPOS, data_buffer_);
			eigenVectorFromBuffer(q_);
			q_ *= M_PI / 180.0;
			redis_.setEigenMatrixString(KEY_JOINT_POSITIONS, q_);

			// Get Puma joint velocities
			puma_robot_->getStatus(GET_JVEL, data_buffer_);
			eigenVectorFromBuffer(dq_);
			dq_ *= M_PI / 180.0;
			redis_.setEigenMatrixString(KEY_JOINT_VELOCITIES, dq_);

			// Get Puma torques
			puma_robot_->getStatus(GET_TORQ, data_buffer_);
			eigenVectorFromBuffer(Gamma_);
			redis_.setEigenMatrixString(KEY_JOINT_TORQUES, Gamma_);

			// Get Puma end effector position/orientation
			puma_robot_->getStatus(GET_IPOS, data_buffer_);
			eigenVectorFromBuffer(x_);
			redis_.setEigenMatrixString(KEY_EE_POS, x_.head(3));
			redis_.setEigenMatrixString(KEY_EE_ORI, x_.tail<4>());
#endif  // CONNECT_SERVER
		}
	} catch (const std::exception& e) {
		std::cerr << e.what() << std::endl;
		stop();
	}
}

