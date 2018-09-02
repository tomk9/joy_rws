#pragma once

#include <QObject>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/math/Q.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <QtGui>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <thread>
#include <chrono>

#include "ui_gamepad.h"

#include <string>

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#define N_EFFECTS 3
#define N_AXES 8
#define N_BUTTONS 11

#define INFO rw::common::Log::log().info()

// void mainLoop();
void rumble(int i);
void initRumble();

class InitROS
{
  public:
	InitROS()
	{
		int argc = 0;
		char *argv[] = {'\0'};
		ros::init(argc, argv, "controller_plugin");
	}
};

/**
 * @brief A plugin for testing controller.
 */
class Controller : public rws::RobWorkStudioPlugin,
				   private Ui::gamepad,
				   InitROS
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "gamepad" FILE "joy_rws.json")
	Q_INTERFACES(rws::RobWorkStudioPlugin)

  public:
	typedef std::pair<rw::math::Q, double> IKSolution;
	//! @brief constructor
	Controller();

	//! @brief destructor
	virtual ~Controller();

	//! @copydoc rws::RobWorkStudioPlugin::open(rw::models::WorkCell* workcell)
	virtual void open(rw::models::WorkCell *workcell);

	//! @copydoc rws::RobWorkStudioPlugin::close()
	virtual void close();

	//! @copydoc rws::RobWorkStudioPlugin::initialize()
	virtual void initialize();

	virtual void update();

	/**
	 * @brief listen for generic event
	 * @param event [in] the event id
	 */
	void genericEventListener(const std::string &event);

	//! @brief listen for key events
	void keyEventListener(int key, Qt::KeyboardModifiers modifier);

	void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
	// rw::models::WorkCell::Ptr _wc;
	void updateGhost();
	void mainLoop();
	rw::math::Q ik(rw::models::Device::Ptr robot, const rw::math::Transform3D<> &t, const rw::kinematics::State &state);
	bool checkCollision(rw::models::Device::Ptr robot, const rw::math::Q &q);
	std::vector<rw::math::Q> expandQ(const std::vector<rw::math::Q> &config);
	// double axes[N_AXES];
	// int buuttons[N_BUTTONS];

  private slots:
	void ObslugaPrzyciskuConnect();

  private:
	/* plugin interface widget */
	Ui::gamepad _ui;

	std::thread mainLoopThread;

	rw::models::WorkCell::Ptr _wc;
	// rw::common::Ptr<rw::models::WorkCell> _wc;
	rw::kinematics::State _state;
	ros::NodeHandle nh_;
	ros::Subscriber joy_sub_;

	// double a_scale_ = 1;
	double axes[N_AXES] = {0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
	int buttons[N_BUTTONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	rw::models::Device::Ptr _robot;
	rw::models::Device::Ptr _ghost;
	rw::models::SerialDevice::Ptr _sdghost;
	rw::invkin::ClosedFormIKSolverUR::Ptr _invkin;
	rw::proximity::CollisionDetector::Ptr _cd;
	rw::kinematics::Frame *_real_tcp_frame;
	rw::kinematics::Frame *_ghost_tcp_frame;
	std::vector<IKSolution> _iksolutions;
	int _steeringMode = 0;

	//   RobotInterface* robotUR5;
};
