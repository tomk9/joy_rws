#include "gamepad.hpp"
#include <rws/RobWorkStudio.hpp>
#include <QMessageBox>

using namespace std;
using namespace rws;
using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace boost;

void mainLoop(){
    while(true){
        Log::infoLog() << "loop" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

Controller::Controller() :
  RobWorkStudioPlugin("Controller", QIcon(":/gamepad.png")) {
  _ui.setupUi(this);
  
  connect(_ui.pushButton_2, SIGNAL(clicked()), this, SLOT(ObslugaPrzyciskuConnect()));
  int argc = 0;
  char* argv[] = {'\0'};
  ros::init(argc, argv, "sts_plugin");
  mainLoopThread = std::thread(mainLoop);
}

Controller::~Controller() {
}

void Controller::initialize() {
  
  //getRobWorkStudio()->genericEvent().add(boost::bind(&RWSPlugin::genericEventListener, this, _1), this);
  //getRobWorkStudio()->keyEvent().add(boost::bind(&RWSPlugin::keyEventListener, this, _1, _2), this);
  
  log().setLevel(Log::Info);
//   robotUR5 = new RobotInterface();
}

void Controller::open(WorkCell* workcell) {
  
  try {
      _wc = workcell;
      _state = getRobWorkStudio()->getState();
      
  } catch (const rw::common::Exception& e) {
      QMessageBox::critical(NULL, "RW Exception", e.what());
  }
}

void Controller::close() {
  /* ... */
}

void Controller::update() {
  /* ... */
}

void Controller::genericEventListener(const std::string & event) {

  /*if (event == "DynamicWorkCellLoaded") {
    
  }*/
}

void Controller::keyEventListener(int key, Qt::KeyboardModifiers modifier) {
  
  /* ... */
}

// void Controller::ObslugaPrzycisku(){
// 	Log::infoLog() << "Hello world!" << endl;

//   rw::models::Device::Ptr _robot = _wc->findDevice("robot");
//   if(_robot != NULL){
//   // rw::math::Q q = _robot->getQ(_state);
//   // Log::infoLog() << q << endl;
//   // rw::math::Q q1 = rw::math::Q(6, 2.3, -1.1, 1.0, 1.3, 1.1, 1.0);
  
//   // _robot->setQ(q1, _state);
//   // getRobWorkStudio()->setState(_state);
//   rwhw::URRTData data = robotUR5->getData();
//   rw::math::Q q = data.qActual;
//         rw::math::Q dq = data.dqActual;
        
//         if (q.size() == 6) {
//         _robot->setQ(q, _state);
//         getRobWorkStudio()->setState(_state);
//         }
//   }

// }

void Controller::ObslugaPrzyciskuConnect(){
  Log::infoLog() << "Connecting..." << endl;
//   if (robotUR5->connect()) {
//     Log::infoLog() << "Connected" << endl;
//   }
}

