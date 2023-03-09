/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/PTPtrajectory.h"

PTPtrajectory::PTPtrajectory()
{
}

PTPtrajectory::PTPtrajectory(const ros::Publisher& trajectoryPub, const ros::Publisher& stopPub, const ros::Publisher& startPub) :
    m_trajectoryPub(trajectoryPub)
    , m_stopPub(stopPub)
    , m_startPub(startPub)
{
    m_target.x = 0.0;
    m_target.y = 0.0;
    m_position.x = 0.0;
    m_position.y = 0.0;
}

void PTPtrajectory::PoseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_position = msg->position;
    if (std::sqrt(std::pow(m_target.x - m_position.x, 2) + std::pow(m_target.y - m_position.y, 2)) < 0.1)
    {
        m_stopPub.publish(std_msgs::Empty());
    }
}

bool PTPtrajectory::TargetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res)
{
    m_target;
    m_target.x = req.x;
    m_target.y = req.y;
    const auto targetDistance = static_cast<float>(std::sqrt(
        std::pow(m_position.x - m_target.x,2) + std::pow(m_position.y - m_target.y,2)));
    
    const auto headingToTarget = std::atan2((m_target.y - m_position.y),
                                            (m_target.x - m_position.x));
    const auto cosinus = cos(headingToTarget);
    const auto sinus = sin(headingToTarget);

    const auto numOfWaypoints = static_cast<int>(targetDistance / WAYPOINT_DISTANCE);

    sgtdv_msgs::Point2DArr waypoints;
    waypoints.points.reserve(numOfWaypoints);

    for (int i = 0; i < numOfWaypoints; i++)
    {
        {
            sgtdv_msgs::Point2D waypoint;
            waypoint.x = m_position.x + (i+1) * WAYPOINT_DISTANCE * cosinus;
            waypoint.y = m_position.y + (i+1) * WAYPOINT_DISTANCE * sinus;
            //waypoint.header.frame_id = "map";
            waypoints.points.emplace_back(waypoint);
        }
    }

    if (numOfWaypoints * WAYPOINT_DISTANCE < targetDistance)
    {
        waypoints.points.push_back(m_target);
    }

    m_trajectoryPub.publish(waypoints);
    m_startPub.publish(std_msgs::Empty());

    return (res.success = waypoints.points.size() > 0);
}