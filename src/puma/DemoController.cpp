/*
 * DemoController.h
 *
 *  Created on: May 10, 2017
 *      Author: Toki Migimatsu
 */

#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "puma/RedisDriver.h"

#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>

using namespace std;

static const std::string kRobotName = "puma";
static const std::string kWorldFile = "resources/puma_demo/world.urdf";
static const std::string kRobotFile = "resources/puma_demo/puma.urdf";
static const HiredisServerInfo kRedisServerInfo = {
	"127.0.0.1",  // hostname
	6379,         // port
	{ 1, 500000 } // timeout = 1.5 seconds
};

static volatile bool g_runloop = true;
static RedisClient *g_redis_client = nullptr;

void stop(int) {
	g_runloop = false;
	if (g_redis_client != nullptr)
		g_redis_client->set(Puma::KEY_CONTROL_MODE, "BREAK");
	cout << "Sending BREAK." << endl;
	exit(1);
}

static void setCtrlCHandler(void (*userCallback)(int)) {
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = userCallback;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char** argv) {

	cout << "This program is a demo Redis controller for the Puma." << endl;

	cout << "Loading URDF world model file: " << kWorldFile << endl;

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	auto redis_client = RedisClient();
	redis_client.serverIs(kRedisServerInfo);

	// Set Ctrl C handler
	setCtrlCHandler(stop);
	g_redis_client = &redis_client;

	// Load robot
	auto robot = new Model::ModelInterface(kRobotFile, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// Create a loop timer
	uintmax_t controller_counter = 0;
	const double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 1 ms pause before starting loop

	/***** Hold *****/

	cout << "HOLD" << endl;

	redis_client.set(Puma::KEY_CONTROL_MODE, "JHOLD");

	/***** Go to home position *****/

	cout << "JGOTO" << endl;

	// Declare control variables
	Eigen::VectorXd Kp(Puma::DOF);
	Eigen::VectorXd Kv(Puma::DOF);
	Eigen::VectorXd q_des(Puma::DOF);
	const double kToleranceInitQ  = 0.1;  // Joint space initialization tolerance
	const double kToleranceInitDq = 0.1;  // Joint space initialization tolerance

	// If lowering gains, set Kp first. If increasing, set Kv first.
	Kp.fill(100);
	Kv.fill(40);

	// Commands MUST be sent as an atomic Redis transaction with MSET
	q_des.setZero();
	q_des << 0.0, 30.0, 0.0, 0.0, -30.0, 0.0;
	q_des *= M_PI / 180.0;
	redis_client.mset({
		{Puma::KEY_CONTROL_MODE, "JGOTO"},
		{Puma::KEY_COMMAND_DATA, RedisClient::encodeEigenMatrixString(q_des)},
		{Puma::KEY_KP, RedisClient::encodeEigenMatrixString(Kp)},
		{Puma::KEY_KV, RedisClient::encodeEigenMatrixString(Kv)}
	});

	// Wait for convergence
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();

		// Read from Redis current sensor values and update the model
		robot->_q = redis_client.getEigenMatrixString(Puma::KEY_JOINT_POSITIONS);
		robot->_dq = redis_client.getEigenMatrixString(Puma::KEY_JOINT_VELOCITIES);
		robot->updateModel();

		// Check for convergence
		if (robot->_q.norm() < kToleranceInitQ && robot->_dq.norm() < kToleranceInitDq) break;

		controller_counter++;
	}

	/***** Track circle *****/

	cout << "GOTO" << endl;

	// Declare control variables
	Eigen::VectorXd x_des(Puma::SIZE_OP_SPACE_TASK);
	Eigen::Vector3d ee_pos_init(0.57, 0.15, 0.32);
	Eigen::Vector3d ee_pos_des = ee_pos_init;
	Eigen::Quaterniond ee_ori_des(1, 0, 0, 0);
	const double kAmplitude = 0.1;
	Kp.fill(100);
	Kv.fill(40);

	// When setting new gains, must change control mode simultaneously.
	x_des << ee_pos_des, ee_ori_des.w(), ee_ori_des.x(), ee_ori_des.y(), ee_ori_des.z();
	redis_client.mset({
		{Puma::KEY_CONTROL_MODE, "GOTO"},
		{Puma::KEY_COMMAND_DATA, RedisClient::encodeEigenMatrixString(x_des)},
		{Puma::KEY_KP, RedisClient::encodeEigenMatrixString(Kp)},
		{Puma::KEY_KV, RedisClient::encodeEigenMatrixString(Kv)}
	});

	// Control loop
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();
		double t_curr = timer.elapsedTime();

		// Read from Redis current sensor values and update the model
		robot->_q = redis_client.getEigenMatrixString(Puma::KEY_JOINT_POSITIONS);
		robot->_dq = redis_client.getEigenMatrixString(Puma::KEY_JOINT_VELOCITIES);
		robot->updateModel();

		// Create a circle trajectory
		ee_pos_des << kAmplitude * cos(t_curr) + ee_pos_init(0),
		              kAmplitude * sin(t_curr) + ee_pos_init(1),
		              ee_pos_init(2);

		// Send command
		x_des << ee_pos_des, ee_ori_des.w(), ee_ori_des.x(), ee_ori_des.y(), ee_ori_des.z();
		redis_client.setEigenMatrixString(Puma::KEY_COMMAND_DATA, x_des);

		// Send desired position for visualization
		redis_client.setEigenMatrixString(Puma::KEY_EE_POS + "_des", ee_pos_des);

		controller_counter++;
	}

	redis_client.set(Puma::KEY_CONTROL_MODE, "BREAK");

	return 0;
}

