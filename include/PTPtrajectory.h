/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#pragma once

// C++
#include <iostream>
#include <cmath>
#include <vector>

// ROS
#include <ros/ros.h>
#include <std_msgs/Empty.h>

// SGT
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <ptp_trajectory/SetTarget.h>

constexpr float WAYPOINT_DISTANCE = 0.2;

class PTPtrajectory
{
    public:
        PTPtrajectory();
        PTPtrajectory(const ros::Publisher& trajectoryPub, const ros::Publisher& stopPub, const ros::Publisher& startPub);
        ~PTPtrajectory() = default;

        void SetSrvServer(ros::ServiceServer& srv)
        {
            m_targetSrv = srv;
        }

        void PoseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
        bool TargetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res);

    private:
    sgtdv_msgs::Point2D m_target;
    sgtdv_msgs::Point2D m_position;
    ros::Publisher m_trajectoryPub;
    ros::Publisher m_stopPub;
    ros::Publisher m_startPub;
    ros::ServiceServer m_targetSrv;
};