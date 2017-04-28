/*  hw2 - main.cpp
This file includes the required code to implement redundant robot control.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 4/18/17
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/GraphicsInterface.h"
#include "simulation/SimulationInterface.h"

#include "simulation/Sai2Simulation.h"

#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

#include "force_sensor/ForceSensorSim.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

// const string world_fname = "resources/hw2/world_2_iiwa.urdf";
// const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string world_fname = "resources/hw2/world_1_puma.urdf";
const string robot_fname = "../resources/puma/puma.urdf";
const string robot_name = "Puma";

// const string robot_name = "Kuka-IIWA";
// const string camera_name = "camera_front";
const string camera_name = "camera_top";
const string ee_link_name = "end-effector";
// const string ee_link_name = "link6";

//typedef std::shared_ptr<cPhantomDevice> cPhantomDevicePtr;


// problem part selection
enum PROBLEM_PART_TYPE {
	PART1=0,
	PART2,
	PART3,
	N_PARTS
};
PROBLEM_PART_TYPE enum_problem_part;
void selectProblemPart(char* input);

// simulation loop
bool fSimulationRunning = false;
void control(Model::ModelInterface* robot, Simulation::SimulationInterface* sim);
void simulation(Model::ModelInterface* robot, Simulation::SimulationInterface* sim);

// void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);
// void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	// get problem part
	if (argc < 2) {
		cout << "Usage: ./hw2-p2 <part: 1 or 2 or 3>" << endl;
		return 0;
	}
	selectProblemPart(argv[1]);

	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::GraphicsInterface(world_fname, Graphics::chai, Graphics::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	// load simulation world
	// auto sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);
	auto sim = new Simulation::SimulationInterface(world_fname, Simulation::sai2simulation, Simulation::urdf, false);

	// set initial condition
	robot->_q << 90/180.0*M_PI,
				-22.5/180.0*M_PI,
				180/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
	// 			180/180.0*M_PI;
	// robot->_q << 125.9/180.0*M_PI,
	// 			39.2/180.0*M_PI,
	// 			-49.2/180.0*M_PI,
	// 			70.0/180.0*M_PI,
	// 			-62.4/180.0*M_PI,
	// 			80.2/180.0*M_PI,
	// 			187.2/180.0*M_PI;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	

    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Model::ModelInterface* robot, Simulation::SimulationInterface* sim) {
// void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {



	auto redis_client = RedisClient();
	HiredisServerInfo serverinfo;
    serverinfo.hostname_ = "127.0.0.1";
    serverinfo.port_ = 6379;
    serverinfo.timeout_ = { 1, 500000 }; // 1.5 seconds
    redis_client.serverIs(serverinfo);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime(); //secs

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());

	// end effector tip position in link frame
	Eigen::Vector3d ee_pos_local;
	ee_pos_local << -0.2, 0.0, 0.0;

	// end effector desired position
	Eigen::Vector3d ee_des_pos, ee_pos;
	Eigen::Matrix3d ee_des_rotation, ee_rotation;
	robot->updateModel();
	robot->position(ee_des_pos, ee_link_name, ee_pos_local);

	Eigen::MatrixXd Jw(3, robot->dof());
	Eigen::MatrixXd Jv(3, robot->dof());
	Eigen::MatrixXd J(6, robot->dof());
	Eigen::MatrixXd MM(3, 3);
	Eigen::MatrixXd MMw(3,3);
	Eigen::MatrixXd L(6,6);
	Eigen::Vector3d LinearVelocity;
	Eigen::Vector3d xd;
	Eigen::VectorXd qd(robot->dof());
	Eigen::MatrixXd J0_swapped(6, robot->dof());
	Eigen::MatrixXd J0(6, robot->dof());
	Eigen::VectorXd p(robot->dof());
	//Eigen::MatrixXd identity(robot->dof(), robot->dof());
	Eigen::MatrixXd identity(robot->dof(), robot->dof());
	identity = Eigen::MatrixXd::Identity(robot->dof(),robot->dof());
	Eigen::MatrixXd tempInverse(robot->dof(), robot->dof());
	Eigen::MatrixXd temp2(robot->dof(), 3);
	Eigen::MatrixXd Jbar(robot->dof(), 6);

	Eigen::Vector3d rotationError;
	Eigen::Vector3d RotationVelocity;
	Eigen::MatrixXd nullSpace(robot->dof(), robot->dof());
	Eigen::MatrixXd prevNullSpace(robot->dof(), robot->dof());
	Eigen::MatrixXd pseudoLambda;
	Eigen::VectorXd vandw(6);
	Eigen::VectorXd posandrot(6);



	xd << -.15, .81, .58;

	// gains
	double kpx = 200.0; // operational space kp
	double kvx = 20.0; // operational space kv
	double kpj = 50.0; // joint space kp
	double kvj = 20.0; // joint space kv
	int i = 0;

	// force sensor: needs Sai2Simulation sim interface type
	// auto force_sensor = new ForceSensorSim(robot_name, ee_link_name, Eigen::Affine3d::Identity(), sim, robot);

	// Eigen::Vector3d sensed_force;
 //    Eigen::Vector3d sensed_moment;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// -------------------------------------------
		// FILL ME IN: set joint torques to simulation

		switch (enum_problem_part) {
			case PART1:
				robot->Jv(Jv, ee_link_name, ee_pos_local);
				robot->J(J, ee_link_name, Eigen::Vector3d::Zero());
				robot->Jw(Jw, ee_link_name);

			    redis_client.getEigenMatrixDerivedString("x_pos", ee_des_pos);
			    redis_client.getEigenMatrixDerivedString("y_pos", ee_des_rotation);
			    ee_des_rotation << 1 , 0, 0,
			    					0, 1, 0,
			    					0, 0, 1;

			    robot->rotation(ee_rotation, ee_link_name);
			    robot->angularVelocity(RotationVelocity, ee_link_name);

				// ee_des_pos << atof(x), atof(y), atof(z);
				
				//std::cout<<ee_des_pos<<std::endl;
				// robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
				// J0.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
				// J0.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw

				robot->position(ee_pos, ee_link_name, ee_pos_local);
				robot->taskInertiaMatrix(MM, Jv);
				robot->taskInertiaMatrix(MMw, Jw);
				robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
				robot->gravityVector(p);
				// Eigen::Quaterniond lambda(rotation); // this converts the rotation matrix to Euler parameters or quaternion
				// double w = lambda.w();
				// double x = lambda.x();
				// double y = lambda.y();
				// double z = lambda.z();
				// Eigen::MatrixXd Eplus(6, 7);
				// Eplus.block(0,0,6,7) = Eigen::MatrixXd::Zero(6,7);
				// Eplus(0,0) = 1;
				// Eplus.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
				// Eigen::MatrixXd eulerparams(3,4);
				// eulerparams << -x, w, -z, y,
				// 				-y, z, w, -x,
				// 				-z, -y, x, w;
				// Eplus.block(3,3, 3, 4) = eulerparams;

				robot->orientationError(rotationError, ee_des_rotation, ee_rotation);


				tau = Jv.transpose() * (MM * (-kpx * (ee_pos - ee_des_pos) - kvx * LinearVelocity )) + p + robot->_M * (-kvj * robot->_dq);
				// tau = p;
				break;
			case PART2:

				robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
				J.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
				J.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw


			    redis_client.getEigenMatrixDerivedString("x_pos", ee_des_pos);
			    redis_client.getEigenMatrixDerivedString("y_pos", ee_des_rotation);

			    robot->rotation(ee_rotation, ee_link_name);
			    robot->angularVelocity(RotationVelocity, ee_link_name);

				robot->position(ee_pos, ee_link_name, ee_pos_local);
				robot->taskInertiaMatrix(L, J);
				robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
				robot->gravityVector(p);

				robot->orientationError(rotationError, ee_des_rotation, ee_rotation);
				posandrot << ee_pos - ee_des_pos, rotationError;
				vandw << LinearVelocity, RotationVelocity;
				robot->dynConsistentInverseJacobian(Jbar, J);
				tau = J.transpose() * (L * (-kpx * (posandrot) - kvx * vandw )) + p + robot->_M * (-kvj * robot->_dq);
				// tau = p;
				break;
			case PART3:
				robot->Jv(Jv, ee_link_name, ee_pos_local);
				robot->J(J, ee_link_name, Eigen::Vector3d::Zero());
				robot->Jw(Jw, ee_link_name);
				J.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
				J.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw

				// qd << robot->_q;
				// qd[1] = -M_PI/8 + (M_PI/8)*sin(2*M_PI*curr_time/10);

			    redis_client.getEigenMatrixDerivedString("x_pos", ee_des_pos);
			    redis_client.getEigenMatrixDerivedString("y_pos", ee_des_rotation);
			    // ee_des_rotation << 1 , 0, 0,
			    // 					0, 1, 0,
			    // 					0, 0, 1;
			    ee_des_pos = ee_des_pos / 5;
			    ee_des_pos(0) = ee_des_pos(0) + 0.3;


			    robot->rotation(ee_rotation, ee_link_name);
			    robot->angularVelocity(RotationVelocity, ee_link_name);

				robot->position(ee_pos, ee_link_name, ee_pos_local);
				robot->taskInertiaMatrix(L, J);
				robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
				robot->gravityVector(p);

				robot->orientationError(rotationError, ee_des_rotation, ee_rotation);
				posandrot << ee_pos - ee_des_pos, rotationError;
				vandw << LinearVelocity, RotationVelocity;
				tau = J.transpose() * (L * (-kpx * (posandrot) - kvx * vandw )) + p + robot->_M * (-kvj * robot->_dq);
				// tau = p;

			 //    robot->rotation(ee_rotation, ee_link_name);
			 //    robot->angularVelocity(RotationVelocity, ee_link_name);

				// robot->position(ee_pos, ee_link_name, ee_pos_local);
				// robot->taskInertiaMatrix(MM, Jv);
				// //robot->taskInertiaMatrix(MMw, Jw);
				// robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
				// robot->gravityVector(p);

				// ee_des_rotation = ee_rotation;
				// robot->orientationError(rotationError, ee_des_rotation, ee_rotation);


				// //cout << Jw.transpose() * (MMw * (-kpx * (rotationError) - kvx * RotationVelocity)) <<endl;
				// robot->dynConsistentInverseJacobian(Jbar, J);
				// //robot->nullspaceMatrix(nullSpace, Jw, prevNullSpace);

				// tempInverse = Jw * Jw.transpose();
				// temp2 = Jw.transpose() * tempInverse.inverse();
				
				// pseudoLambda = J * robot->_M_inv * J.transpose();
				// //pseudoLambda = pseudoLambda.transpose() * (pseudoLambda * pseudoLambda.transpose()).inverse();
				// Jbar = robot->_M_inv * J.transpose() * pseudoLambda.transpose() * (pseudoLambda * pseudoLambda.transpose()).inverse();
				// nullSpace = identity - (J.transpose() * Jbar.transpose());


				// // Jw = Jw * nullSpace;
				// // robot->taskInertiaMatrix(MMw, Jw);
				
				// //tau = Jv.transpose() * (MM * (-kpx * (ee_pos - ee_des_pos) - kvx * LinearVelocity )) + p + (identity - (Jv.transpose() * Jbar.block(0,0,robot->dof(), 3).transpose())) * Jw.transpose() * (MMw * (-10 * (rotationError) - 10 * RotationVelocity));//(robot->_M * (-kpj* (robot->_q - qd) - kvj * (robot->_dq)));
				// tau = Jv.transpose() * (MM * (-kpx * (ee_pos - ee_des_pos) - kvx * LinearVelocity )) + p + (robot->_M * (-kpj* (robot->_q - qd) - kvj * (robot->_dq))) + nullSpace * Jw.transpose() * (MMw * (-10 * (rotationError) - 20 * RotationVelocity));//(robot->_M * (-kpj* (robot->_q - qd) - kvj * (robot->_dq)));
				// // tau = p;
				// //cout << Jw.transpose() * (MMw * (-10 * (rotationError) - 20 * RotationVelocity)) <<endl<<endl;
				// cout<<nullSpace<<endl <<endl;
				// prevNullSpace = nullSpace;
				// // for(int i = 0; i < robot->dof(); i++) {
				// // 	if(tau(i) > 10) tau(i) = 10;
				// // }
				break;
			default:
				tau.setZero();
				break;
		}


		if(i > 10){
			// force_sensor->update();
			// force_sensor->getForce(sensed_force);
	  //       force_sensor->getMoment(sensed_moment);
	  //       cout<< sensed_force << ", " <<sensed_moment << endl <<endl;
	   		i++;

	    }
		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot, Simulation::SimulationInterface* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void selectProblemPart(char* input) {
	switch (input[0]) {
		case '1':
			enum_problem_part = PART1;
			break;
		case '2':
			enum_problem_part = PART2;
			break;
		case '3':
			enum_problem_part = PART3;
			break;
		default:
			cout << "Usage: ./hw2-p2 <part: 1 or 2 or 3>" << endl;
			exit(0);
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
