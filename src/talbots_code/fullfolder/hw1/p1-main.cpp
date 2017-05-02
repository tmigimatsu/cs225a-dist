/*  hw1 - main.cpp
This file includes the required code to implement problem 2.2.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 4/9/17
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/GraphicsInterface.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/hw1/world.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string camera_name = "camera_front";
// const string camera_name = "camera_top";
const string ee_link_name = "link6";

// simulation loop
bool fSimulationRunning = false;
void simulation(Model::ModelInterface* robot);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::GraphicsInterface(world_fname, Graphics::chai, Graphics::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	// set initial condition
	robot->_q << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
	// 80.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				70.0/180.0*M_PI,
				-62.4/180.0*M_PI,
				80.2/180.0*M_PI,
				187.2/180.0*M_PI;
	robot->updateModel();
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot);
	
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

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs

	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd Jv;
	Eigen::MatrixXd Jw;

	//double x, y, z,l0,l1,l2,l3;
	double dx, dy, dz,dl0,dl1,dl2,dl3;
	dx = 0;
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate joint velocity to joint positions
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		robot->_q += robot->_dq*loop_dt;

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update kinematic models
		robot->updateModel();
		//std::cout<<curr_time<<endl;
		// ------------------------------------
		// FILL ME IN: set new joint velocities
		// y = 0.5 + 0.1*cos(2*M_PI*curr_time/5);
		// z = 0.65 - 0.05*cos(4*M_PI*curr_time/5);
		// l0 = (1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5));
		// l1 = (1/sqrt(2))*cos((M_PI/4)*cos(2*M_PI*curr_time/5));
		// l2 = (1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5));
		// l3 = (1/sqrt(2))*cos((M_PI/4)*cos(2*M_PI*curr_time/5));

		dy = -0.04*M_PI*sin(2*M_PI*curr_time/5);
		dz = 0.04*M_PI*sin(4*M_PI*curr_time/5);
		dl0 = (1.0/sqrt(2.0))*cos((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);
		dl1 = (-1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);
		dl2 = (1/sqrt(2))*cos((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);
		dl3 = (-1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);

		Eigen::MatrixXd J0_swapped(6, robot->dof());
		robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
		Eigen::MatrixXd J0(6, robot->dof());
		J0.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
		J0.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw
		
		Eigen::MatrixXd temp = (J0 * robot->_M_inv * J0.transpose());
		// Eigen::MatrixXd temp2 = temp.array().inverse();
		Jbar = robot->_M_inv * J0.transpose() * temp.inverse();

		// Eigen::MatrixXd temp = (J0.array().inverse() * robot->_M * J0.transpose().array().inverse());
		// Eigen::MatrixXd temp2 = temp;
		// Jbar = robot->_M_inv * J0.transpose() * temp2;

		Eigen::Matrix3d rotation;
		robot->rotation(rotation, ee_link_name); // this fetches the rotation matrix describing the end effector frame in the base frame
		Eigen::Quaterniond lambda(rotation); // this converts the rotation matrix to Euler parameters or quaternion
		double w = lambda.w();
		double x = lambda.x();
		double y = lambda.y();
		double z = lambda.z();

		Eigen::MatrixXd Eplus(6, 7);
		Eplus.block(0,0,6,7) = Eigen::MatrixXd::Zero(6,7);
		Eplus(0,0) = 1;
		Eplus.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
		Eigen::MatrixXd eulerparams(3,4);
		eulerparams << -x, w, -z, y,
						-y, z, w, -x,
						-z, -y, x, w;
		Eplus.block(3,3, 3, 4) = eulerparams;
		
		Eigen::VectorXd xd;
		xd.resize(7);
		xd << dx, dy, dz, dl0, dl1, dl2, dl3;
		// Jbar * Eplus * xd;
		robot->_dq = Jbar * Eplus * xd;
		std::cout<<Jbar <<"\n"<<std::endl;
		// robot->_dq = Eigen::VectorXd::Zero(robot->dof());
		// robot->_dq << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
		// ------------------------------------

		// update last time
		last_time = curr_time;
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW1", NULL, NULL);
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
