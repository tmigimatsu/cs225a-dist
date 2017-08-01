// This example application runs a controller for the Sawyer

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "sawyer/RedisDriver.h"

#include <iostream>
#include <string>

#include <signal.h>
static volatile bool g_runloop = true;
void stop(int signal){ g_runloop = false; }

const std::string kWorldFile = "resources/sawyer_hold_pos/world.urdf";
const std::string kRobotFile = "resources/sawyer_hold_pos/sawyer.urdf";
const std::string kRobotName = "sawyer";

const int kp_joint = 30;
const int kv_joint = 20;

unsigned long long controller_counter = 0;

int main() {
	std::cout << "Loading URDF world model file: " << kWorldFile << std::endl;

	// Start redis client
	RedisClient redis;
	redis.connect();

	// Set up signal handler
	signal(SIGABRT, &stop);
	signal(SIGTERM, &stop);
	signal(SIGINT, &stop);

	// Load robot
	auto robot = new Model::ModelInterface(kRobotFile, Model::rbdl, Model::urdf, true);

	// Read from Redis
	robot->_q = redis.getEigenMatrix(Sawyer::KEY_JOINT_POSITIONS);
	robot->_dq = redis.getEigenMatrix(Sawyer::KEY_JOINT_VELOCITIES);

	// Update the model
	robot->updateModel();

	// Initialize controller variables
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(Sawyer::DOF);
	Eigen::VectorXd q_initial = robot->_q;

	// Create a loop timer
	LoopTimer timer;
	timer.setLoopFrequency(1e3);   // 1 KHz
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 1 ms pause before starting loop

	// Loop until interrupt
	while (g_runloop) {
		// Wait for next scheduled loop
		timer.waitForNextLoop();

		// Read from Redis
		robot->_q = redis.getEigenMatrix(Sawyer::KEY_JOINT_POSITIONS);
		robot->_dq = redis.getEigenMatrix(Sawyer::KEY_JOINT_VELOCITIES);

		// Update the model
		robot->updateModel();

		// Joint control
		command_torques = -kp_joint * (robot->_q - q_initial) - kv_joint * robot->_dq;

		// Send torques to robot
		redis.setEigenMatrix(Sawyer::KEY_COMMAND_TORQUES, command_torques);

		controller_counter++;
	}

	// Clear torques
    command_torques.setZero();
    redis.setEigenMatrix(Sawyer::KEY_COMMAND_TORQUES, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
