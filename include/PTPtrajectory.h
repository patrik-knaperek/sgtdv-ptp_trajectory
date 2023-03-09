/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

// C++
//#include <math.h>
#include <iostream>
#include <cmath>
//#include <unistd.h>
#include <vector>
//#include <map>
//#include <chrono>

//#include "opencv2/core/core.hpp"

// ROS
#include <ros/ros.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

// SGT
//#include "Messages.h"
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <ptp_trajectory/SetTarget.h>

constexpr float WAYPOINT_DISTANCE = 0.5;

class PTPtrajectory
{
    public:
        PTPtrajectory();
        PTPtrajectory(ros::Publisher& trajectoryPublisher);
        ~PTPtrajectory() = default;

        void SetSrvServer(ros::ServiceServer& srv)
        {
            m_targetSrv = srv;
        }

        void PoseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
        bool TargetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res);

    private:
    sgtdv_msgs::Point2D m_position;
    ros::Publisher m_trajectoryPub;
    ros::ServiceServer m_targetSrv;

    
};