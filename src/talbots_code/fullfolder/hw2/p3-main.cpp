/*  hw2 - main.cpp
This file includes the required code to implement unified end-effector position-force control.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 4/16/17
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

#include "force_sensor/ForceSensorSim.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/hw2/world_3_iiwa.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
// const string camera_name = "camera_front";
// const string camera_name = "camera_top";
// const string camera_name = "camera_back_top";
const string camera_name = "camera_back_side";
const string ee_link_name = "link6";

// simulation loop
bool fSimulationRunning = false;
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);

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

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);
    // set co-efficient of restition to zero for force control
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(0.0);
    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	// set initial condition
	robot->_q << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				70.0/180.0*M_PI,
				-62.4/180.0*M_PI,
				80.2/180.0*M_PI,
				187.2/180.0*M_PI;
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
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime(); //secs

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::MatrixXd Jbar(robot->dof(), 6);
	Eigen::MatrixXd J0_swapped(6, robot->dof());
	Eigen::MatrixXd J0(6, robot->dof());

	Eigen::VectorXd vandw(6);
	Eigen::VectorXd posandrot(6);
	Eigen::Vector3d ee_des_pos, ee_pos;
	Eigen::Matrix3d ee_rotation;
	Eigen::Quaterniond ee_des_rotation;
	Eigen::Vector3d rotationError;
	Eigen::Vector3d RotationVelocity;
	Eigen::Vector3d LinearVelocity;
	Eigen::VectorXd g;
	Eigen::VectorXd p;
	Eigen::Vector3d ee_pos_local;
	Eigen::MatrixXd L(6, 6);
	Eigen::VectorXd ee_acceleration;
	
	double x, y, z,l0,l1,l2,l3;
	double dx, dy, dz,dl0,dl1,dl2,dl3;
	double ddx, ddy, ddz,ddL0,ddL1,ddL2,ddL3;

	// gains
	double kpx = 50.0; // operational space kp
	double kvx = 20.0; // operational space kv
	double kpj = 50.0; // joint space kp
	double kvj = 20.0; // joint space kv



	robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
	J0.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
	J0.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw

	// force sensor: needs Sai2Simulation sim interface type
	auto force_sensor = new ForceSensorSim(robot_name, ee_link_name, Eigen::Affine3d::Identity(), sim, robot);
	Eigen::Vector3d sensed_force;
    Eigen::Vector3d sensed_moment;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		
		y = 0.5 + 0.1*cos(2*M_PI*curr_time/5);
		z = 0.65 - 0.05*cos(4*M_PI*curr_time/5);
		l0 = (1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5));
		l1 = (1/sqrt(2))*cos((M_PI/4)*cos(2*M_PI*curr_time/5));
		l2 = (1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5));
		l3 = (1/sqrt(2))*cos((M_PI/4)*cos(2*M_PI*curr_time/5));

		dy = -0.04*M_PI*sin(2*M_PI*curr_time/5);
		dz = 0.04*M_PI*sin(4*M_PI*curr_time/5);
		dl0 = (1.0/sqrt(2.0))*cos((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);
		dl1 = (-1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);
		dl2 = (1/sqrt(2))*cos((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);
		dl3 = (-1/sqrt(2))*sin((M_PI/4)*cos(2*M_PI*curr_time/5))*(sin(2*M_PI*curr_time/5)*(-M_PI/4)*2*M_PI/5);

		ee_des_pos << 0, y, z;
		ee_des_rotation.w() = l0;
		ee_des_rotation.x() = l1;
		ee_des_rotation.y() = l2;
		ee_des_rotation.z() = l3;


		robot->J(J0_swapped, ee_link_name, Eigen::Vector3d::Zero()); // get the swapped basic Jacobian from the model interface
		J0.block(0, 0, 3, robot->dof()) = J0_swapped.block(3, 0, 3, robot->dof()); // assign Jv
		J0.block(3, 0, 3, robot->dof()) = J0_swapped.block(0, 0, 3, robot->dof()); // assign Jw


		robot->rotation(ee_rotation, ee_link_name);
	    robot->angularVelocity(RotationVelocity, ee_link_name);

	    Eigen::Quaterniond quarot(ee_rotation);

		robot->position(ee_pos, ee_link_name, ee_pos_local);
		robot->taskInertiaMatrix(L, J0);
		robot->linearVelocity(LinearVelocity, ee_link_name, ee_pos_local);
		robot->gravityVector(p);

		robot->dynConsistentInverseJacobian(Jbar, J0);

		ddx = 0;
		ddy = -(0.016*(M_PI)*M_PI)* cos(2*M_PI*curr_time/5);
		ddz = 0.032*(M_PI)*M_PI * cos(4*M_PI*curr_time/5);
		ddL0 = ((-sqrt(2)*M_PI*M_PI*M_PI)/50)*cos(M_PI/4*cos(2*M_PI*curr_time/5))*cos(2*M_PI*curr_time/5) + ((sqrt(2)*M_PI*M_PI*M_PI*M_PI)/200)*sin(M_PI/4*(cos(2*M_PI*curr_time/5)))*sin(2*M_PI*curr_time/5)*sin(2*M_PI*curr_time/5);
		ddL1 = ((sqrt(2)*M_PI*M_PI*M_PI)/50)*sin(M_PI/4*cos(2*M_PI*curr_time/5))*cos(2*M_PI*curr_time/5) - ((sqrt(2)*M_PI*M_PI*M_PI*M_PI)/200)*cos(M_PI/4*(cos(2*M_PI*curr_time/5)))*sin(2*M_PI*curr_time/5)*sin(2*M_PI*curr_time/5);
		ddL2 = ddL0;//((-sqrt(2)*M_PI*M_PI*M_PI)/50)*cos(M_PI/4*cos(2*M_PI*curr_time/5))*cos(2*M_PI*curr_time/5) + ((sqrt(2)*M_PI*M_PI*M_PI*M_PI)/200)*sin(M_PI/4*(cos(2*M_PI*curr_time/5)))*sin(2*M_PI*curr_time/5)*sin(2*M_PI*curr_time/5);
		ddL3 = ddL1;//((sqrt(2)*M_PI*M_PI*M_PI)/50)*sin(M_PI/4*cos(2*M_PI*curr_time/5))*cos(2*M_PI*curr_time/5) - ((sqrt(2)*M_PI*M_PI*M_PI*M_PI)/200)*cos(M_PI/4*(cos(2*M_PI*curr_time/5)))*sin(2*M_PI*curr_time/5)*sin(2*M_PI*curr_time/5);

		ee_acceleration.resize(7);
		ee_acceleration << ddx, ddy, ddz, ddL0, ddL1, ddL2, ddL3;



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


		robot->orientationError(rotationError, ee_des_rotation, quarot);
		posandrot << ee_pos - ee_des_pos, rotationError;

		Eigen::VectorXd xd;
		xd.resize(7);
		xd << dx, dy, dz, dl0, dl1, dl2, dl3;

		// cout<<ee_des_rotation.w() <<endl<<ee_des_rotation.x()<<endl<<ee_des_rotation.y() <<endl<<ee_des_rotation.z() <<endl<<endl;
		// cout<<quarot.w() <<endl<<quarot.x()<<endl<<quarot.y() <<endl<<quarot.z() <<endl<<endl;
		vandw << LinearVelocity, RotationVelocity;
		robot->gravityVector(g);
		p = Jbar.transpose() * g;	
		Eigen::MatrixXd omega = Eigen::MatrixXd::Zero(6,6);
		omega(1,1) = 1;
		omega(2,2) = 1;
		omega(3,3) = 1;
		Eigen::MatrixXd omegainv = Eigen::MatrixXd::Zero(6,6);
		omegainv(0,0) = 1;
		omegainv(4,4) = 1;
		omegainv(5,5) = 1;
		Eigen::VectorXd forcedesired = Eigen::VectorXd::Zero(6);
		forcedesired(0) = 10;
		tau = J0.transpose() * ((L * (omega* (Eplus*ee_acceleration -kpx * (posandrot) - kvx * (vandw - Eplus * xd) )) + p) + (L * (omegainv* (Eplus*ee_acceleration -kpx * (posandrot) - kvx * (vandw - Eplus * xd) )) + forcedesired)) + (Eigen::MatrixXd::Identity(6,6) - J0.transpose() * Jbar.transpose())*robot->_M * (-kvj * robot->_dq);


		// update force sensor readings
		force_sensor->update();
		force_sensor->getForce(sensed_force);
        force_sensor->getMoment(sensed_moment);
        cout<< sensed_force << ", " <<sensed_moment << endl <<endl;

		// -------------------------------------------
		// FILL ME IN: set joint torques to simulation


		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
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
