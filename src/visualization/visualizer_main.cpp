// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.
// NOTE: this application only works in 2d.

#include <model/ModelInterface.h>
#include <simulation/SimulationInterface.h>
#include <graphics/GraphicsInterface.h>
#include <graphics/ChaiGraphics.h>
#include "redis/RedisClient.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "uiforce/UIForceWidget.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

string world_file = "";
string robot_file = "";
string robot_name = "";
const string camera_name = "camera_fixed";

// redis keys: 
// NOTE: keys are formatted to be: key_prepend::<robot-name>::<KEY>
const std::string key_prepend = "cs225a::robot::";
// - write:
const std::string JOINT_INTERACTION_TORQUES_COMMANDED_KEY = "::actuators::fgc_interact";
// - read:
const std::string JOINT_ANGLES_KEY  = "::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "::sensors::dq";

// function to parse command line arguments
void parseCommandline(int argc, char** argv);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// callback when user scrolls
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fRotPanTilt = false;
bool fZoom = false;
double zoomSpeed = 0.0;
bool fRobotLinkSelect = false;

int main(int argc, char** argv) {
	parseCommandline(argc, argv);
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1"; //"172.24.68.64";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// load graphics scene
	// auto graphics_int = make_shared<Graphics::GraphicsInterface>(world_file, Graphics::chai, Graphics::urdf, true);
	auto graphics_int = new Graphics::GraphicsInterface(world_file, Graphics::chai, Graphics::urdf, true);
	Graphics::ChaiGraphics* graphics;
	graphics = dynamic_cast<Graphics::ChaiGraphics *>(graphics_int->_graphics_internal);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	auto x_traj = new chai3d::cMultiSegment();
	auto x_des_traj = new chai3d::cMultiSegment();
	const int LEN_TRAJECTORY = 100;
	x_traj->m_name = key_prepend + robot_name + "::tasks::ee_pos_traj";
	x_des_traj->m_name = key_prepend + robot_name + "::tasks::ee_pos_des_traj";
	x_traj->newVertex(0.0, 0.0, 0.0);
	x_des_traj->newVertex(0.0, 0.0, 0.0);
	for (int i = 0; i < LEN_TRAJECTORY; i++) {
		x_traj->newVertex(0.0, 0.0, 0.0);
		x_traj->newSegment(0, 0);
		x_des_traj->newVertex(0.0, 0.0, 0.0);
		x_des_traj->newSegment(0, 0);
	}
	x_traj->setLineColor(chai3d::cColorf(1.0, 1.0, 1.0, 1.0));
	x_traj->setLineWidth(2.0);
	x_des_traj->setLineColor(chai3d::cColorf(1.0, 0.0, 0.0, 1.0));
	x_des_traj->setLineWidth(2.0);
	graphics->_world->addChild(x_traj);
	graphics->_world->addChild(x_des_traj);

	// load robots
	auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl, Model::urdf, false);

	// create a UI widget to apply mouse forces to the robot
	UIForceWidget force_widget(robot_name, robot.get(), graphics);
	force_widget.setEnable(false);

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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS225a", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);
	glfwSetScrollCallback(window, mouseScroll);

	// cache and temp variables
	double last_cursorx, last_cursory;
	Eigen::VectorXd interaction_torques;
	Eigen::Vector3d x_des, x_des_prev, x, x_prev;
	int idx_traj = 0, idx_traj_next = 1, idx_des_traj = 0, idx_des_traj_next = 1;
	redis_client.getEigenMatrixDerivedString(key_prepend+robot_name+"::tasks::ee_pos", x);
	for (unsigned int i = 0; i < graphics->_world->getNumChildren(); ++i) {
		auto graphics_obj = graphics->_world->getChild(i);
		if (graphics_obj->m_name == key_prepend+robot_name+"::tasks::ee_pos_traj") {
			auto x_traj = dynamic_cast<chai3d::cMultiSegment *>(graphics_obj);
			x_traj->m_vertices->setLocalPos(0, x(0), x(1), x(2));
		} else if (graphics_obj->m_name == key_prepend+robot_name+"::tasks::ee_pos_des_traj") {
			auto x_des_traj = dynamic_cast<chai3d::cMultiSegment *>(graphics_obj);
			x_des_traj->m_vertices->setLocalPos(0, x(0), x(1), x(2));
		}
	}
	x_prev = x;
	x_des_prev = x_des;

    // while window is open:
    while (!glfwWindowShouldClose(window))
	{
		// read from Redis
		redis_client.getEigenMatrixDerivedString(key_prepend+robot_name+JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(key_prepend+robot_name+JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.getEigenMatrixDerivedString(key_prepend+robot_name+"::tasks::ee_pos_des", x_des);
		redis_client.getEigenMatrixDerivedString(key_prepend+robot_name+"::tasks::ee_pos", x);
		for (unsigned int i = 0; i < graphics->_world->getNumChildren(); ++i) {
			auto graphics_obj = graphics->_world->getChild(i);
			if (graphics_obj->m_name == key_prepend+robot_name+"::tasks::ee_pos_des") {
				graphics_obj->setLocalPos(chai3d::cVector3d(x_des));
			} else if (graphics_obj->m_name == key_prepend+robot_name+"::tasks::ee_pos_traj") {
				if ((x - x_prev).norm() < 0.01) continue;
				int idx_traj_next_2 = (idx_traj_next + 1) % LEN_TRAJECTORY;
				auto x_traj = dynamic_cast<chai3d::cMultiSegment *>(graphics_obj);
				x_traj->m_vertices->setLocalPos(idx_traj_next, x(0), x(1), x(2));
				x_traj->m_segments->setVertices(idx_traj, idx_traj, idx_traj_next);
				x_traj->m_segments->setVertices(idx_traj_next, idx_traj_next_2, idx_traj_next_2);
				idx_traj = idx_traj_next;
				idx_traj_next = idx_traj_next_2;
				x_prev = x;
			} else if (graphics_obj->m_name == key_prepend+robot_name+"::tasks::ee_pos_des_traj") {
				if ((x_des - x_des_prev).norm() < 0.01) continue;
				int idx_des_traj_next_2 = (idx_des_traj_next + 1) % LEN_TRAJECTORY;
				auto x_des_traj = dynamic_cast<chai3d::cMultiSegment *>(graphics_obj);
				x_des_traj->m_vertices->setLocalPos(idx_des_traj_next, x_des(0), x_des(1), x_des(2));
				x_des_traj->m_segments->setVertices(idx_des_traj, idx_des_traj, idx_des_traj_next);
				x_des_traj->m_segments->setVertices(idx_des_traj_next, idx_des_traj_next_2, idx_des_traj_next_2);
				idx_des_traj = idx_des_traj_next;
				idx_des_traj_next = idx_des_traj_next_2;
				x_des_prev = x_des;
			}
		}

		// update transformations
		robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot.get());
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

	    // poll for events
	    glfwPollEvents();

	    // move scene camera as required
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			// camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
			// TODO: the above doesn't work as intended because Chai treats the lookat
			// vector as a direction vector in the local frame, rather than as a lookat point
			camera_pos = m_pan*(camera_pos);
			camera_lookat = m_pan*(camera_lookat);
			// TODO: the above fix is a HUGE hack. Think about improving this.
	    }
	    if (fZoom) {
			camera_pos = camera_pos + 0.04*camera_lookat*zoomSpeed;
			fZoom = false;
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	    if (fRobotLinkSelect) {
			//activate widget
			force_widget.setEnable(true);
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);
			int viewx, viewy;
			viewx = floor(cursorx/wwidth_scr * wwidth_pix);
			viewy = floor(cursory/wheight_scr * wheight_pix);
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;
			if (cursorx > 0 && cursory > 0) {
				force_widget.setInteractionParams(camera_name, viewx, wheight_pix-viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
	    } else {
			force_widget.setEnable(false);
	    }
		// get UI torques
		force_widget.getUIJointTorques(interaction_torques);
		//write to redis
		redis_client.setEigenMatrixDerivedString(key_prepend+robot_name+JOINT_INTERACTION_TORQUES_COMMANDED_KEY, interaction_torques);
	}

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: visualizer <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
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

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		default:
			break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			// TODO: move link select to shift + left click
			// TODO: set menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset) {
	fZoom = true;
	zoomSpeed = yoffset;
}
