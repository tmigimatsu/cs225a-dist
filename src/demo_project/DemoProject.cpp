#include "DemoProject.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include <signal.h>
static volatile bool runloop = true;
void stop(int) { runloop = false; }

template<typename Derived>
inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

using namespace std;

void DemoProject::readRedisValues() {
	// read from Redis current sensor values
	redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	// Get current simulation timestamp from Redis
	redis_client.getCommandIs(TIMESTAMP_KEY, redis_buf);
	t_curr = stod(redis_buf);

	// Read in KP and KV from Redis (can be changed on the fly in Redis)
	redis_client.getCommandIs(KP_POSITION_KEY, redis_buf);
	kp_pos = stoi(redis_buf);
	redis_client.getCommandIs(KV_POSITION_KEY, redis_buf);
	kv_pos = stoi(redis_buf);
	redis_client.getCommandIs(KP_ORIENTATION_KEY, redis_buf);
	kp_ori = stoi(redis_buf);
	redis_client.getCommandIs(KV_ORIENTATION_KEY, redis_buf);
	kv_ori = stoi(redis_buf);
	redis_client.getCommandIs(KP_JOINT_KEY, redis_buf);
	kp_joint = stoi(redis_buf);
	redis_client.getCommandIs(KV_JOINT_KEY, redis_buf);
	kv_joint = stoi(redis_buf);
}

void DemoProject::writeRedisValues() {
	redis_client.setEigenMatrixDerivedString("cs225a::robot::kuka_iiwa::tasks::ee_pos", x);
	redis_client.setEigenMatrixDerivedString("cs225a::robot::kuka_iiwa::tasks::ee_pos_des", x_des);

	// Send torques
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
}

void DemoProject::updateModel() {
	// Update the model
	robot->updateModel();

	// Forward kinematics
	robot->position(x, "link6", Eigen::Vector3d::Zero());
	robot->linearVelocity(dx, "link6", Eigen::Vector3d::Zero());

	// Jacobians
	robot->Jv(Jv, "link6", Eigen::Vector3d::Zero());
	robot->nullspaceMatrix(N, Jv);

	// Dynamics
	robot->taskInertiaMatrixWithPseudoInv(Lambda_x, Jv);
	robot->gravityVector(g);
}

DemoProject::controller_status_t DemoProject::computeJointSpaceControlTorques() {
	/**
	 * JOINT SPACE CONTROL
	 * -------------------
	 * Controller to initialize robot to desired joint position.
	 * Uses kp = 400 and kv = 40 by default.
	 */

	// Break if the robot has converged to q_initial
	q_err = robot->_q - q_des;
	dq_err = robot->_dq - dq_des;
	if (q_err.norm() < TOLERANCE_Q_INIT && dq_err.norm() < TOLERANCE_DQ_INIT) {
		return FINISHED;
	}

	// Compute torques
	ddq = -kp_joint * q_err - kv_joint * dq_err;
	command_torques = robot->_M * ddq + g;
	return RUNNING;
}

DemoProject::controller_status_t DemoProject::computeOperationalSpaceControlTorques() {
	/**
	 * OPERATIONAL SPACE CONTROL
	 * -------------------------
	 * Controller to implement for HW2
	 */

	// PD position control with velocity saturation
	x_err = x - x_des;
	// dx_err = dx - dx_des;
	// ddx = -kp_pos * x_err - kv_pos * dx_err;
	dx_des = -(kp_pos / kv_pos) * x_err;
	double v = MAX_VELOCITY / dx_des.norm();
	if (v > 1) v = 1;
	dx_err = dx - v * dx_des;
	ddx = -kv_pos * dx_err;

	// Nullspace posture control and damping
	q_err = robot->_q - q_des;
	dq_err = robot->_dq - dq_des;
	ddq = -kp_joint * q_err - kv_joint * dq_err;

	// Control torques
	F_x = Lambda_x * ddx;
	F_posture = robot->_M * ddq;
	command_torques = Jv.transpose() * F_x + N.transpose() * F_posture + g;

	return RUNNING;
}

void DemoProject::initialize() {
	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	redis_client.serverIs(redisServerInfo);

	// Create a loop timer
	timer.setLoopFrequency(CONTROL_FREQ);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(INITIALIZATION_PAUSE); // 1 ms pause before starting loop

	// Set gains in Redis if not initialized
	if (!redis_client.getCommandIs(KP_POSITION_KEY)) {
		redis_buf = to_string(kp_pos);
		redis_client.setCommandIs(KP_POSITION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_POSITION_KEY)) {
		redis_buf = to_string(kv_pos);
		redis_client.setCommandIs(KV_POSITION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KP_ORIENTATION_KEY)) {
		redis_buf = to_string(kp_ori);
		redis_client.setCommandIs(KP_ORIENTATION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_ORIENTATION_KEY)) {
		redis_buf = to_string(kv_ori);
		redis_client.setCommandIs(KV_ORIENTATION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KP_JOINT_KEY)) {
		redis_buf = to_string(kp_joint);
		redis_client.setCommandIs(KP_JOINT_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_JOINT_KEY)) {
		redis_buf = to_string(kv_joint);
		redis_client.setCommandIs(KV_JOINT_KEY, redis_buf);
	}
}

void DemoProject::runLoop() {
	while (runloop) {
		// wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();
		controller_counter++;

		readRedisValues();
		updateModel();

		switch (controller_state) {
			case REDIS_SYNCHRONIZATION:
				if (isnan(robot->_q)) continue;
				cout << "Redis synchronized. Switching to joint space controller." << endl;
				controller_state = JOINT_SPACE_INITIALIZATION;
				break;

			case JOINT_SPACE_INITIALIZATION:
				if (computeJointSpaceControlTorques() == FINISHED) {
					cout << "Joint position initialized. Switching to operational space controller." << endl;
					controller_state = DemoProject::OP_SPACE_POSITION_CONTROL;
				};
				break;

			case OP_SPACE_POSITION_CONTROL:
				computeOperationalSpaceControlTorques();
				break;

			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				runloop = false;
				command_torques.setZero();
				continue;
		}

		if (isnan(command_torques)) {
			cout << "NaN command torques. Sending zero torques to robot." << endl;
			command_torques.setZero();
		}
		writeRedisValues();
	}

	// Zero out torques before quitting
	command_torques.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
}

int main(int argc, char** argv) {

	// Parse command line
	if (argc != 4) {
		cout << "Usage: demo_app <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	string world_file(argv[1]);
	// argument 2: <path-to-robot.urdf>
	string robot_file(argv[2]);
	// argument 3: <robot-name>
	string robot_name(argv[3]);

	// Load robot
	cout << "Loading robot: " << robot_file << endl;
	auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// Start controller app
	cout << "Initializing app with " << robot_name << endl;
	DemoProject app(move(robot), robot_name);
	app.initialize();
	cout << "App initialized. Waiting for Redis synchronization." << endl;
	app.runLoop();

	return 0;
}


