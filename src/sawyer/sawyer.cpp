#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>
static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

// Location of URDF files specifying world and robot information
static string world_file = "";
static string robot_file = "";
static string robot_name = "";

static unsigned long long controller_counter = 0;

// Redis keys:
// - write:
static std::string JOINT_TORQUES_COMMANDED_KEY = "";

// - read:
static std::string JOINT_ANGLES_KEY  = "";
static std::string JOINT_VELOCITIES_KEY = "";
static std::string TIMESTAMP_KEY = "";
static std::string KP_POSITION_KEY = "";
static std::string KV_POSITION_KEY = "";
static std::string KP_ORIENTATION_KEY = "";
static std::string KV_ORIENTATION_KEY = "";
static std::string KP_JOINT_KEY = "";
static std::string KV_JOINT_KEY = "";
static std::string KP_JOINT_INIT_KEY = "";
static std::string KV_JOINT_INIT_KEY = "";

// Function to parse command line arguments
void parseCommandline(int argc, char** argv);

int main(int argc, char** argv) {

	// Parse command line and set redis keys
	parseCommandline(argc, argv);
	JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc";
	JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q";
	JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq";
	TIMESTAMP_KEY               = "cs225a::robot::" + robot_name + "::timestamp";
	KP_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kp_pos";
	KV_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kv_pos";
	KP_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kp_ori";
	KV_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kv_ori";
	KP_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kp_joint";
	KV_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kv_joint";
	KP_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kp_joint_init";
	KV_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kv_joint_init";

	cout << "Loading URDF world model file: " << world_file << endl;

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// Load robot
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();
	const int dof = robot->dof();
	// Create a loop timer
	const double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 1 ms pause before starting loop
	
	Eigen::VectorXd command_torques(dof);

	// While window is open:
	while (runloop) {
		// wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();

		// read from Redis current sensor values
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

		// Update the model
		robot->updateModel();	
		command_torques.setZero();
		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}
	
	command_torques.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	return 0;
}


//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: hw2 <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	world_file = string(argv[1]);
	// argument 2: <path-to-robot.urdf>
	robot_file = string(argv[2]);
	// argument 3: <robot-name>
	robot_name = string(argv[3]);
}
