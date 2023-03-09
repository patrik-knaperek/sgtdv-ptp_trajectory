/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/PTPtrajectory.h"

PTPtrajectory::PTPtrajectory()
{
}

PTPtrajectory::PTPtrajectory(ros::Publisher& trajectoryPublisher) :
    m_trajectoryPub(trajectoryPublisher)
{
    m_position.x = 0.0;
    m_position.y = 0.0;
}

void PTPtrajectory::PoseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_position = msg->position;
}

bool PTPtrajectory::TargetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res)
{
    sgtdv_msgs::Point2D target;
    target.x = req.x;
    target.y = req.y;
    const auto targetDistance = static_cast<float>(std::sqrt(
        std::pow(m_position.x - target.x,2) + std::pow(m_position.y - target.y,2)));
    
    const auto headingToTarget = std::atan2((target.y - m_position.y),
                                            (target.x - m_position.x));
    const auto cosinus = cos(headingToTarget);
    const auto sinus = sin(headingToTarget);

    const auto numOfWaypoints = static_cast<int>(targetDistance / WAYPOINT_DISTANCE);
    //std::cout << "num of waypoints: " << numOfWaypoints << std::endl;
    sgtdv_msgs::Point2DArr waypoints;
    waypoints.points.reserve(numOfWaypoints);

    for (int i = 0; i < numOfWaypoints; i++)
    {
        {
            sgtdv_msgs::Point2D waypoint;
            waypoint.x = m_position.x + (i+1) * WAYPOINT_DISTANCE * cosinus;
            waypoint.y = m_position.y + (i+1) * WAYPOINT_DISTANCE * sinus;
            waypoint.header.frame_id = "map";
            waypoints.points.emplace_back(waypoint);
            //std::cout << "added waypoint: " << waypoints.points[i] << std::endl;
        }
    }

    if (numOfWaypoints * WAYPOINT_DISTANCE < targetDistance)
    {
        waypoints.points.push_back(target);
    }

    m_trajectoryPub.publish(waypoints);

    return (res.success = waypoints.points.size() > 0);
}