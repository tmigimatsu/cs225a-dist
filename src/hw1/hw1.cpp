#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;

// Location of URDF files specifying world and robot information
const string world_file = "resources/hw0/world.urdf";
const string robot_file = "resources/hw1/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc";

// - read:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::" + robot_name + "::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::" + robot_name + "::sensors::dq";
const std::string JOINT_KP_KEY = "cs225a::robot::" + robot_name + "::gui::joint_kp";
const std::string JOINT_KV_KEY = "cs225a::robot::" + robot_name + "::gui::joint_kv";

// -debug:
const std::string JOINT_TASK_Q_DESIRED_KEY = "cs225a::robot::" + robot_name + "::joint_task_q_desired";


int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
	
	robot->updateModel();
	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd g = Eigen::VectorXd::Zero(dof); // Empty Gravity Vector for use
	Eigen::VectorXd b = Eigen::VectorXd::Zero(dof); // Empty Coriolis/Centrifugal Force Vector
	Eigen::VectorXd joint_task_desired_position(dof), joint_task_torques(dof);


	// Joint Space Control, set the desired joint position and write the initial value to redis
	joint_task_desired_position << 90.0/180.0*M_PI, 30.0/180.0*M_PI, 0.0, 60.0/180.0*M_PI, 0.0, 90.0/180.0*M_PI, 0.0;
	redis_client.setEigenMatrixDerivedString(JOINT_TASK_Q_DESIRED_KEY,joint_task_desired_position);

	// Gains, set initial value in redis
	Eigen::VectorXd tmp = Eigen::VectorXd(1);
	double joint_kp = 0;
	double joint_kv = 0;
	tmp(0) = joint_kp;
	redis_client.setEigenMatrixDerivedString(JOINT_KP_KEY, tmp);
	tmp(0) = joint_kv;
	redis_client.setEigenMatrixDerivedString(JOINT_KV_KEY, tmp);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();

		// read from Redis current sensor values
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

		// update the model 20 times slower (for computational speed)
		if(controller_counter % 20 == 0)
		{
			robot->updateModel();
		}

		//----- Joint controller 
		// Get the Q Desired from Redis (can be changed on the fly in Redis)
		redis_client.getEigenMatrixDerivedString(JOINT_TASK_Q_DESIRED_KEY, joint_task_desired_position);

		// Read in KP and KV from Redis (can be changed on the fly in Redis)
		redis_client.getEigenMatrixDerivedString(JOINT_KP_KEY, tmp);
		joint_kp = tmp(0);
		redis_client.getEigenMatrixDerivedString(JOINT_KV_KEY, tmp);
		joint_kv = tmp(0);
		joint_task_torques = 0; // Fill this in with Control and Feedback Linearization (mass matrix)

		//------ Final torques
		command_torques = joint_task_torques; // Fill in with gravity vector and/or coriolis/centrifugal force
		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    return 0;
}

