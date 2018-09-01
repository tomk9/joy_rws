#pragma once

#include <QObject>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models/WorkCell.hpp>
#include <QtGui>
#include <ros/ros.h>
#include <thread>
#include <chrono>

#include "ui_gamepad.h"


/**
 * @brief A plugin for testing controller.
 */
class Controller: public rws::RobWorkStudioPlugin,
private Ui::gamepad {
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "gamepad" FILE "joy_rws.json")
	Q_INTERFACES(rws::RobWorkStudioPlugin)

public:
	//! @brief constructor
	Controller();

	//! @brief destructor
	virtual ~Controller();

	//! @copydoc rws::RobWorkStudioPlugin::open(rw::models::WorkCell* workcell)
	virtual void open(rw::models::WorkCell* workcell);

	//! @copydoc rws::RobWorkStudioPlugin::close()
	virtual void close();

	//! @copydoc rws::RobWorkStudioPlugin::initialize()
	virtual void initialize();

	virtual void update();

	/**
	 * @brief listen for generic event
	 * @param event [in] the event id
	 */
	void genericEventListener(const std::string& event);

	//! @brief listen for key events
	void keyEventListener(int key, Qt::KeyboardModifiers modifier);

private slots:
void ObslugaPrzyciskuConnect();

private:
  /* plugin interface widget */
	Ui::gamepad _ui;

	std::thread mainLoopThread;
  
  rw::models::WorkCell* _wc;
  rw::kinematics::State _state;

//   RobotInterface* robotUR5;
};
