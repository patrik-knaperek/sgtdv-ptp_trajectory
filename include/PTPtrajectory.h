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
#include <ptp_trajectory/GoRectangle.h>
#include <ptp_trajectory/SetTarget.h>

class PTPtrajectory
{
    public:
        static constexpr float WAYPOINT_DISTANCE = 0.1;
        static constexpr float MIN_DISTANCE = 0.1;

    public:
        PTPtrajectory();
        PTPtrajectory(const ros::Publisher& trajectoryPub, const ros::Publisher& stopPub, const ros::Publisher& startPub);
        ~PTPtrajectory() = default;

        void SetSrvServers(ros::ServiceServer& rectSrv, ros::ServiceServer& targetSrv)
        {
            m_rectangleSrv = rectSrv;
            m_targetSrv = targetSrv;
        }

        void PoseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
        bool RectangleCallback(ptp_trajectory::GoRectangle::Request& req, ptp_trajectory::GoRectangle::Response& res);
        bool TargetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res);

    private:
    const sgtdv_msgs::Point2DArr::Ptr ComputeWaypoints(const sgtdv_msgs::Point2D &start, const sgtdv_msgs::Point2D &target) const;
    void UpdateTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &waypoints);
    void PublishTrajectory() const;

    sgtdv_msgs::Point2D m_target;
    sgtdv_msgs::Point2DArr m_trajectory;
    sgtdv_msgs::Point2D m_position;
    ros::Publisher m_trajectoryPub;
    ros::Publisher m_stopPub;
    ros::Publisher m_startPub;
    ros::ServiceServer m_rectangleSrv;
    ros::ServiceServer m_targetSrv;
};