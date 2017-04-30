
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

static string world_file = "";
static string robot_file = "";
static string robot_name = "";

// redis keys: 
// NOTE: keys are formatted to be: REDIS_KEY_PREFIX::<robot-name>::<KEY>
static const std::string REDIS_KEY_PREFIX = "cs225a::robot::";
// - read:
static std::string JOINT_TORQUES_COMMANDED_KEY             = "::actuators::fgc";
static std::string JOINT_INTERACTION_TORQUES_COMMANDED_KEY = "::actuators::fgc_interact";
// - write:
static std::string JOINT_ANGLES_KEY     = "::sensors::q";
static std::string JOINT_VELOCITIES_KEY = "::sensors::dq";
static std::string SIM_TIMESTAMP_KEY    = "::timestamp";

static const double SENSOR_WRITE_FREQ = 1000;
static const double SIM_FREQ = 10000;  // set the simulation frequency. Ideally 10kHz
static const double TIMER_FREQ = 1000; // set 
static const unsigned int NS_INITIAL_WAIT = 10 * 1e6; // 10ms pause before starting loop

// function to parse command line arguments
static void parseCommandline(int argc, char** argv);

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
	LoopTimer timer;
	timer.setLoopFrequency(SIM_FREQ);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(10 * 1e6); // 10 ms pause before starting loop

	Eigen::VectorXd robot_torques = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd robot_torques_interact = Eigen::VectorXd::Zero(robot->dof());
	robot->_q.setZero();
	robot->_dq.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, robot_torques);
	redis_client.setEigenMatrixDerivedString(JOINT_INTERACTION_TORQUES_COMMANDED_KEY, robot_torques_interact);

	double time_sensor_last = timer.elapsedTime();
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		redis_client.getEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, robot_torques);
		redis_client.getEigenMatrixDerivedString(JOINT_INTERACTION_TORQUES_COMMANDED_KEY, robot_torques_interact);
		sim->setJointTorques(robot_name, robot_torques + robot_torques_interact);

		// update simulation by 1ms
		sim->integrate(1.0 / SIM_FREQ);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		
		double time_sensor = timer.elapsedTime();
		if (time_sensor - time_sensor_last >= 1.0 / SENSOR_WRITE_FREQ) {
			// write joint kinematics to redis
			redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
			redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

			redis_client.setCommandIs(SIM_TIMESTAMP_KEY, std::to_string(timer.elapsedSimTime()));
			time_sensor_last = time_sensor;
		}

	}
	return 0;
}

//------------------------------------------------------------------------------
static void parseCommandline(int argc, char** argv) {
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

	// Set up Redis keys
	JOINT_TORQUES_COMMANDED_KEY = REDIS_KEY_PREFIX + robot_name + JOINT_TORQUES_COMMANDED_KEY;
	JOINT_INTERACTION_TORQUES_COMMANDED_KEY = REDIS_KEY_PREFIX + robot_name + JOINT_INTERACTION_TORQUES_COMMANDED_KEY;
	JOINT_ANGLES_KEY            = REDIS_KEY_PREFIX + robot_name + JOINT_ANGLES_KEY;
	JOINT_VELOCITIES_KEY        = REDIS_KEY_PREFIX + robot_name + JOINT_VELOCITIES_KEY;
	SIM_TIMESTAMP_KEY           = REDIS_KEY_PREFIX + robot_name + SIM_TIMESTAMP_KEY;
}
