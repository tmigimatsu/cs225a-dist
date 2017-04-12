#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

// Location of URDF files specifying world and robot information
static string world_file = "";
static string robot_file = "";
static string robot_name = "";

unsigned long long controller_counter = 0;

// Redis keys:
// - write:
static std::string JOINT_TORQUES_COMMANDED_KEY = "";

// - read:
static std::string JOINT_ANGLES_KEY  = "";
static std::string JOINT_VELOCITIES_KEY = "";
static std::string JOINT_KP_KEY = "";
static std::string JOINT_KV_KEY = "";

// -debug:
static std::string JOINT_TASK_Q_DESIRED_KEY = "";

// Function to parse command line arguments
void parseCommandline(int argc, char** argv);

int main(int argc, char** argv) {

	// Parse command line and set redis keys
	parseCommandline(argc, argv);
	JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc";
	JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q";
	JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq";
	JOINT_KP_KEY                = "cs225a::robot::" + robot_name + "::tasks::joint_kp";
	JOINT_KV_KEY                = "cs225a::robot::" + robot_name + "::tasks::joint_kv";
	JOINT_TASK_Q_DESIRED_KEY    = "cs225a::robot::" + robot_name + "::tasks::q_desired";

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

	// Initialize control variables for joint space control
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd joint_task_desired_position(dof);

	// Set the desired joint position and write the initial value to redis
	joint_task_desired_position.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_TASK_Q_DESIRED_KEY, joint_task_desired_position);

	// Gains, set initial value in redis if they don't exist already
	string redis_buf;
	double joint_kp = 0;
	double joint_kv = 0;
	if (!redis_client.getCommandIs(JOINT_KP_KEY)) {
		redis_buf = to_string(joint_kp);
		redis_client.setCommandIs(JOINT_KP_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(JOINT_KV_KEY)) {
		redis_buf = to_string(joint_kv);
		redis_client.setCommandIs(JOINT_KV_KEY, redis_buf);
	}

	// Create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// While window is open:
	while (runloop) {

		// Wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();

		// Read from Redis current sensor values
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

		// Update the model 20 times slower (for computational speed)
		if (controller_counter % 20 == 0) {
			robot->updateModel();
		}

		//----- Joint space controller
		// Get the Q Desired from Redis (can be changed on the fly in Redis)
		redis_client.getEigenMatrixDerivedString(JOINT_TASK_Q_DESIRED_KEY, joint_task_desired_position);

		// Read in KP and KV from Redis (can be changed on the fly in Redis)
		redis_client.getCommandIs(JOINT_KP_KEY, redis_buf);
		joint_kp = stoi(redis_buf);
		redis_client.getCommandIs(JOINT_KV_KEY, redis_buf);
		joint_kv = stoi(redis_buf);

		//------ Final torques
		command_torques.setZero();
		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	command_torques.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	return 0;
}


//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: hw1 <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
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
