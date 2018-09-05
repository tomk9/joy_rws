#include "RobotInterface.hpp"


    RobotInterface::RobotInterface(){

    }

    bool RobotInterface::connect(){
        _ur_log = new rwhw::UniversalRobotsRTLogging();
        _ur_log->connect("192.168.2.10");
        _ur_log->start();

        _ur_cb = new rwhw::URCallBackInterface();
        _ur_cb->connect("192.168.2.10");
        _ur_cb->startCommunication("192.168.2.50", 33333);
        return true;
    }

    rwhw::URRTData RobotInterface::getData(){
        // ros::spinOnce();
        rwhw::URRTData data = _ur_log->getLastData();
        return data;
    }

    bool RobotInterface::move_to_q(const rw::math::Q& q, double speed, double timeout){
      bool ret = false;
  
      // ros::Time t_now = ros::Time::now();
      // ros::Time timer = t_now + ros::Duration(timeout);
      
      //if (_ur && _ur->isConnected()) {
      if (_ur_cb) {
        //ret = _ur_cb->moveQ(q, speed);
        _ur_cb->moveQ(q, speed);
        ret = true;
        
      //   if (timeout > 0.0) {
      //     bool expired = false;
      //     bool reached = false;
      //     while (!expired && !reached) {
      //       expired = (ros::Time::now() > timer);
      //       reached = robotReachedQ(q);
      //     }
      //     if (expired) ret = false;
      //   }
      }
  
      return ret;
    }

  

    bool RobotInterface::robotReachedQ(const rw::math::Q& q, double eps) {
      bool ret = false;
      
      if (_ur_log) {
        rwhw::URRTData data = _ur_log->getLastData();
        rw::math::Q q0 = data.qActual;
        
        double d = rw::math::MetricUtil::dist2<rw::math::Q>(q, q0);
        ret = (d < eps);
     }

      return ret;
    } 

    bool RobotInterface::robotConnect() {
        if (_ur_log) {
            _ur_log->stop();
            delete _ur_log;
        }
        _ur_log = new rwhw::UniversalRobotsRTLogging();
        _ur_log->connect("192.168.2.10");
        _ur_log->start();
        
        if (_ur_cb) {
            _ur_cb->stopCommunication();
            delete _ur_cb;
        }
        
        _ur_cb = new rwhw::URCallBackInterface();
        _ur_cb->connect("192.168.2.10");
        _ur_cb->startCommunication("192.168.2.50", 33333);
        return true;
    }

    RobotInterface::~RobotInterface(){

    }
   
