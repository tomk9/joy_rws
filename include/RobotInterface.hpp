#ifndef ROBOTINTERFACE_HPP
#define ROBOTINTERFACE_HPP

#include <rwhw/universalrobots/URPrimaryInterface.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URCommon.hpp>
#include <rwhw/universalrobots/UniversalRobotsData.hpp>
#include <rwhw/universalrobots/URMessage.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <ros/ros.h>
#include <rw/math/MetricUtil.hpp>



class RobotInterface {
    public:
        RobotInterface();
        bool connect();
        rwhw::URRTData getData();
        bool move_to_q(const rw::math::Q& q, double speed, double timeout);
        bool robotConnect();
        ~RobotInterface();

    protected:
        bool robotReachedQ(const rw::math::Q& q, double eps=1e-3);

    private:
        rwhw::UniversalRobotsRTLogging* _ur_log;
        rwhw::URCallBackInterface* _ur_cb;


};


#endif