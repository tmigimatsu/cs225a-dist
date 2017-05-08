#include "DemoProject.h"

#include <iostream>
#include <fstream>

#include <signal.h>
static volatile bool g_runloop = true;
void stop(int) { g_runloop = false; }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

using namespace std;

/**
 * DemoProject::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void DemoProject::readRedisValues() {
	// read from Redis current sensor values
	redis_client_.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	redis_client_.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	// Get current simulation timestamp from Redis
	redis_client_.getCommandIs(TIMESTAMP_KEY, redis_buf_);
	t_curr_ = stod(redis_buf_);

	// Read in KP and KV from Redis (can be changed on the fly in Redis)
	redis_client_.getCommandIs(KP_POSITION_KEY, redis_buf_);
	kp_pos_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KV_POSITION_KEY, redis_buf_);
	kv_pos_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KP_ORIENTATION_KEY, redis_buf_);
	kp_ori_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KV_ORIENTATION_KEY, redis_buf_);
	kv_ori_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KP_JOINT_KEY, redis_buf_);
	kp_joint_ = stoi(redis_buf_);
	redis_client_.getCommandIs(KV_JOINT_KEY, redis_buf_);
	kv_joint_ = stoi(redis_buf_);
}

/**
 * DemoProject::writeRedisValues()
 * -------------------------------
 * Send all write keys to Redis.
 */
void DemoProject::writeRedisValues() {
	// Send end effector position and desired position
	redis_client_.setEigenMatrixDerivedString(EE_POSITION_KEY, x_);
	redis_client_.setEigenMatrixDerivedString(EE_POSITION_DESIRED_KEY, x_des_);

	// Send torques
	redis_client_.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
}

/**
 * DemoProject::updateModel()
 * --------------------------
 * Update the robot model and all the relevant member variables.
 */
void DemoProject::updateModel() {
	// Update the model
	robot->updateModel();

	// Forward kinematics
	robot->position(x_, "link6", Eigen::Vector3d::Zero());
	robot->linearVelocity(dx_, "link6", Eigen::Vector3d::Zero());

	// Jacobians
	robot->Jv(Jv_, "link6", Eigen::Vector3d::Zero());
	robot->nullspaceMatrix(N_, Jv_);

	// Dynamics
	robot->taskInertiaMatrixWithPseudoInv(Lambda_x_, Jv_);
	robot->gravityVector(g_);
}

/**
 * DemoProject::computeJointSpaceControlTorques()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
DemoProject::ControllerStatus DemoProject::computeJointSpaceControlTorques() {
	// Finish if the robot has converged to q_initial
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	if (q_err.norm() < kToleranceInitQ && dq_err.norm() < kToleranceInitDq) {
		return FINISHED;
	}

	// Compute torques
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;
	command_torques_ = robot->_M * ddq + g_;
	return RUNNING;
}

/**
 * DemoProject::computeOperationalSpaceControlTorques()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
DemoProject::ControllerStatus DemoProject::computeOperationalSpaceControlTorques() {
	// PD position control with velocity saturation
	Eigen::Vector3d x_err = x_ - x_des_;
	// Eigen::Vector3d dx_err = dx_ - dx_des_;
	// Eigen::Vector3d ddx = -kp_pos_ * x_err - kv_pos_ * dx_err_;
	dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
	double v = kMaxVelocity / dx_des_.norm();
	if (v > 1) v = 1;
	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
	Eigen::Vector3d ddx = -kv_pos_ * dx_err;

	// Nullspace posture control and damping
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;

	// Control torques
	Eigen::Vector3d F_x = Lambda_x_ * ddx;
	Eigen::VectorXd F_posture = robot->_M * ddq;
	command_torques_ = Jv_.transpose() * F_x + N_.transpose() * F_posture + g_;

	return RUNNING;
}

/**
 * public DemoProject::initialize()
 * --------------------------------
 * Initialize timer and Redis client
 */
void DemoProject::initialize() {
	// Create a loop timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	redis_client_.serverIs(kRedisServerInfo);

	// Set up optitrack
	optitrack_ = make_unique<OptiTrackClient>();
	// optitrack_->openConnection("123.45.67.89");
	optitrack_->openCsv("../resources/optitrack_120.csv");

	// Demo usage for OptiTrackClient
	// TODO: Remove
	while (g_runloop) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if (!optitrack_->getFrame()) continue;

		cout << "Timestamp: " << optitrack_->frameTime() << endl;
		cout << "Rigid body positions:" << endl;
		for (auto& pos : optitrack_->pos_rigid_bodies_) {
			cout << '\t' << pos.transpose() << endl;
		}
		cout << "Marker positions:" << endl;
		for (auto& pos : optitrack_->pos_single_markers_) {
			cout << '\t' << pos.transpose() << endl;
		}
		cout << endl;
	}

	// Set gains in Redis if not initialized
	if (!redis_client_.getCommandIs(KP_POSITION_KEY)) {
		redis_buf_ = to_string(kp_pos_);
		redis_client_.setCommandIs(KP_POSITION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_POSITION_KEY)) {
		redis_buf_ = to_string(kv_pos_);
		redis_client_.setCommandIs(KV_POSITION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KP_ORIENTATION_KEY)) {
		redis_buf_ = to_string(kp_ori_);
		redis_client_.setCommandIs(KP_ORIENTATION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_ORIENTATION_KEY)) {
		redis_buf_ = to_string(kv_ori_);
		redis_client_.setCommandIs(KV_ORIENTATION_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KP_JOINT_KEY)) {
		redis_buf_ = to_string(kp_joint_);
		redis_client_.setCommandIs(KP_JOINT_KEY, redis_buf_);
	}
	if (!redis_client_.getCommandIs(KV_JOINT_KEY)) {
		redis_buf_ = to_string(kv_joint_);
		redis_client_.setCommandIs(KV_JOINT_KEY, redis_buf_);
	}
}

/**
 * public DemoProject::runLoop()
 * -----------------------------
 * DemoProject state machine
 */
void DemoProject::runLoop() {
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer_.waitForNextLoop();
		++controller_counter_;

		// Get latest sensor values from Redis and update robot model
		readRedisValues();
		updateModel();

		switch (controller_state_) {
			// Wait until valid sensor values have been published to Redis
			case REDIS_SYNCHRONIZATION:
				if (isnan(robot->_q)) continue;
				cout << "Redis synchronized. Switching to joint space controller." << endl;
				controller_state_ = JOINT_SPACE_INITIALIZATION;
				break;

			// Initialize robot to default joint configuration
			case JOINT_SPACE_INITIALIZATION:
				if (computeJointSpaceControlTorques() == FINISHED) {
					cout << "Joint position initialized. Switching to operational space controller." << endl;
					controller_state_ = DemoProject::OP_SPACE_POSITION_CONTROL;
				};
				break;

			// Control end effector to desired position
			case OP_SPACE_POSITION_CONTROL:
				computeOperationalSpaceControlTorques();
				break;

			// Invalid state. Zero torques and exit program.
			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				g_runloop = false;
				command_torques_.setZero();
				break;
		}

		// Check command torques before sending them
		if (isnan(command_torques_)) {
			cout << "NaN command torques. Sending zero torques to robot." << endl;
			command_torques_.setZero();
		}

		// Send command torques
		writeRedisValues();
	}

	// Zero out torques before quitting
	command_torques_.setZero();
	redis_client_.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
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


