#ifndef FORCE_SENSOR_DISPLAY_H
#define FORCE_SENSOR_DISPLAY_H

#include "ForceSensorSim.h"
#include <graphics/ChaiGraphics.h>
#include <chai3d.h>

// Class to display force and moments sensed by a force/moment sensor
class ForceSensorDisplay {
public:
	ForceSensorDisplay(ForceSensorSim* sensor_sim, Graphics::ChaiGraphics* graphics)
	: _sensor_sim(sensor_sim), _graphics(graphics)
	{
		// initialize display lines
		_display_line_force = new chai3d::cShapeLine();
		_display_line_force->setShowEnabled(false);
		_display_line_force->m_colorPointA.setGreenYellowGreen();
		_display_line_force->m_colorPointB.setGreenYellowGreen();
		_graphics->_world->addChild(_display_line_force);
		_display_line_force->setLineWidth(4.0);

		_display_line_moment = new chai3d::cShapeLine();
		_display_line_moment->setShowEnabled(false);
		_display_line_moment->m_colorPointA.setBrownMaroon();
		_display_line_moment->m_colorPointB.setBrownMaroon();
		_graphics->_world->addChild(_display_line_moment);
		_display_line_moment->setLineWidth(4.0);
	}

	void update() {
		// TODO: guard against _sensor_sim = NULL?
		auto robot_model = _sensor_sim->_model;
		// get common point A
		Eigen::Vector3d epointA;
		robot_model->position(epointA, _sensor_sim->_data->_link_name, _sensor_sim->_data->_transform_in_link.translation());

		// force:
		_display_line_force->m_pointA = chai3d::cVector3d(epointA);
		// get point B
		Eigen::Vector3d sensor_force;
		_sensor_sim->getForce(sensor_force);
		_display_line_force->m_pointB = chai3d::cVector3d(epointA + sensor_force*0.01); //100 to 1 scale down in length
		// set show true
		_display_line_force->setShowEnabled(true);

		// moment:
		_display_line_moment->m_pointA = chai3d::cVector3d(epointA);
		// get point B
		Eigen::Vector3d sensor_moment;
		_sensor_sim->getMoment(sensor_moment);
		_display_line_moment->m_pointB = chai3d::cVector3d(epointA + sensor_moment*0.1); //100 to 1 scale down in length
		// set show true
		_display_line_moment->setShowEnabled(true);
	}

public:
	// a line to be displayed when a contact force is active
	chai3d::cShapeLine* _display_line_force;

	// a line to be displayed when a contact moment is active
	chai3d::cShapeLine* _display_line_moment;

	// simulated force sensor. TODO: should be a const pointer
	ForceSensorSim* _sensor_sim;

	// handle to graphics interface to query interaction state change
	Graphics::ChaiGraphics* _graphics;
};

#endif //FORCE_SENSOR_DISPLAY_H
