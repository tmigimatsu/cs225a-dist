
#include <model/ModelInterface.h>
#include <simulation/SimulationInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

string world_file = "";
string robot_file = "";
string robot_name = "";

// redis keys: 
// NOTE: keys are formatted to be: key_preprend::<robot-name>::<KEY>
const std::string key_preprend = "cs225a::robot::";
// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY = "::actuators::fgc";
const std::string JOINT_INTERACTION_TORQUES_COMMANDED_KEY = "::actuators::fgc_interact";
// - write:
const std::string JOINT_ANGLES_KEY  = "::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "::sensors::dq";
const std::string SIM_TIMESTAMP_KEY = "::timestamp";

// function to parse command line arguments
void parseCommandline(int argc, char** argv);

int main(int argc, char** argv) {
	parseCommandline(argc, argv);
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load simulation world
	auto sim = new Simulation::SimulationInterface(world_file, Simulation::sai2simulation, Simulation::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// create a loop timer
	double sim_freq = 1000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop


	Eigen::VectorXd robot_torques(robot->dof());
	Eigen::VectorXd robot_torques_interact(robot->dof());
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		redis_client.getEigenMatrixDerivedString(key_preprend+robot_name+JOINT_TORQUES_COMMANDED_KEY, robot_torques);
		redis_client.getEigenMatrixDerivedString(key_preprend+robot_name+JOINT_INTERACTION_TORQUES_COMMANDED_KEY, robot_torques_interact);
		sim->setJointTorques(robot_name, robot_torques+robot_torques_interact);

		// update simulation by 1ms
		sim->integrate(1/sim_freq);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		
		// write joint kinematics to redis
		redis_client.setEigenMatrixDerivedString(key_preprend+robot_name+JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixDerivedString(key_preprend+robot_name+JOINT_VELOCITIES_KEY, robot->_dq);

		redis_client.setCommandIs(key_preprend+robot_name+SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

	}
	return 0;
}

//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: simulator <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
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
