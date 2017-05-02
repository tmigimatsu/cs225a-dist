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

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/hw2/world_1_puma.urdf";
const string robot_fname = "../resources/puma/puma.urdf";
const string robot_name = "Puma";
// const string camera_name = "camera_front";
const string camera_name = "camera_top";
const string ee_link_name = "end-effector";

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
	auto sim = new Simulation::SimulationInterface(world_fname, Simulation::sai2simulation, Simulation::urdf, false);

	// set initial condition
	robot->_q << 90/180.0*M_PI,
				-22.5/180.0*M_PI,
				180/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
				180/180.0*M_PI;
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
	robot->updateModel();
	robot->position(ee_des_pos, ee_link_name, ee_pos_local);

	Eigen::MatrixXd Jv(3, robot->dof());
	Eigen::MatrixXd J(6, robot->dof());
	Eigen::MatrixXd MM(3, 3);
	Eigen::Vector3d LinearVelocity;
	Eigen::Vector3d xd;
	Eigen::VectorXd qd(robot->dof());
	Eigen::MatrixXd J0_swapped(6, robot->dof());
	Eigen::MatrixXd J0(6, robot->dof());
	Eigen::VectorXd p(robot->dof());
	Eigen::MatrixXd identity(robot->dof(), robot->dof());
	identity = Eigen::MatrixXd::Identity(robot->dof(),robot->dof());
	Eigen::MatrixXd tempInverse(robot->dof(), robot->dof());
	Eigen::MatrixXd temp2(robot->dof(), 3);
	Eigen::MatrixXd Jbar(robot->dof(), 3);
	Eigen::VectorXd g;


	xd << -.15, .81, .58;

	// gains
	double kpx = 50.0; // operational space kp
	double kvx = 20.0; // operational space kv
	double kpj = 50.0; // joint space kp
	double kvj = 20.0; // joint space kv


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
				robot->dynConsistentInverseJacobian(Jbar, Jv);

				
				// robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
				// J0.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
				// J0.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw

				robot->position(ee_pos, ee_link_name, ee_pos_local);
				robot->taskInertiaMatrix(MM, Jv);
				robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
				robot->gravityVector(g);
				p = Jbar.transpose() * g;
				tau = Jv.transpose() * (MM * (-kpx * (ee_pos - ee_des_pos) - kvx * LinearVelocity ) + p) + robot->_M *(-kvj * robot->_dq);
				// tau = p;
				break;
			case PART2:
				robot->Jv(Jv, ee_link_name, ee_pos_local);
				robot->J(J, ee_link_name, Eigen::Vector3d::Zero());
				robot->dynConsistentInverseJacobian(Jbar, Jv);
				qd << robot->_q;
				qd[1] = -M_PI/8 + (M_PI/8)*sin(2*M_PI*curr_time/10);
				std::cout<<qd<<"\n"<<std::endl;
				// robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
				// J0.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
				// J0.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw


				robot->position(ee_pos, ee_link_name, ee_pos_local);
				robot->taskInertiaMatrix(MM, Jv);
				robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
				robot->gravityVector(p);
				tempInverse = Jv * Jv.transpose();
				temp2 = Jv.transpose() * tempInverse.inverse();
				robot->gravityVector(g);
				p = Jbar.transpose() * g;
				tau = Jv.transpose() * (MM * (-kpx * (ee_pos - ee_des_pos) - kvx * LinearVelocity ) + p) + (identity - (Jv.transpose() * temp2.transpose()))*(robot->_M * (-kpj* (robot->_q - qd) - kvj * (robot->_dq)));
				// tau = p;
				break;
			case PART3:
				robot->Jv(Jv, ee_link_name, ee_pos_local);
				robot->J(J, ee_link_name, Eigen::Vector3d::Zero());
				qd << robot->_q;
				qd[1] = -M_PI/8 + (M_PI/8)*sin(2*M_PI*curr_time/10);
				std::cout<<qd<<"\n"<<std::endl;
				// robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
				// J0.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
				// J0.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw


				robot->position(ee_pos, ee_link_name, ee_pos_local);
				robot->taskInertiaMatrix(MM, Jv);
				robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
				robot->gravityVector(p);
				robot->dynConsistentInverseJacobian(Jbar, Jv);

				robot->gravityVector(g);
				p = Jbar.transpose() * g;
				tau = Jv.transpose() * (MM * (-kpx * (ee_pos - ee_des_pos) - kvx * LinearVelocity ) + p) + (identity - (Jv.transpose() * Jbar.block(0,0,robot->dof(), 3).transpose()))*(robot->_M * (-kpj* (robot->_q - qd) - kvj * (robot->_dq)));
				// tau = p;
				break;
			default:
				tau.setZero();
				break;
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
