/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/PTPtrajectory.h"

PTPtrajectory::PTPtrajectory()
{
}

PTPtrajectory::PTPtrajectory(const ros::Publisher& trajectoryPub) 
: m_trajectoryPub(trajectoryPub)
{
    m_target.x = 0.0;
    m_target.y = 0.0;
    m_position.x = 0.0;
    m_position.y = 0.0;
}

void PTPtrajectory::PoseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_position = msg->position;

    if (!m_moved)
    {
            if (std::sqrt(std::pow(m_target.x - m_position.x, 2) + std::pow(m_target.y - m_position.y, 2)) > 4.0 * MIN_DISTANCE)
                m_moved = true;
    }
    else if (std::sqrt(std::pow(m_target.x - m_position.x, 2) + std::pow(m_target.y - m_position.y, 2)) < MIN_DISTANCE)
    {
        if (!ros::service::call("pathTracking/stop", m_srvMsg))
	    {
		    ROS_ERROR("Service \"pathTracking/stop\" failed");
	    }
        m_trajectory.points.clear();
    }
}

bool PTPtrajectory::RectangleCallback(ptp_trajectory::GoRectangle::Request& req, ptp_trajectory::GoRectangle::Response& res)
{
    sgtdv_msgs::Point2D target1;
    target1.x = m_position.x + req.a / 2;
    target1.y = m_position.y;
    UpdateTrajectory(ComputeWaypoints(m_position, target1));

    sgtdv_msgs::Point2D target2;
    target2.x = target1.x;
    target2.y = m_position.y + req.b * (req.right ? -1 : 1);
    UpdateTrajectory(ComputeWaypoints(target1, target2));
    
    target1 = target2;
    target2.x -= req.a;
    UpdateTrajectory(ComputeWaypoints(target1, target2));

    target1 = target2;
    target2.y = m_position.y;
    UpdateTrajectory(ComputeWaypoints(target1, target2));

    target1 = target2;
    target2.x = m_position.x - MIN_DISTANCE;
    UpdateTrajectory(ComputeWaypoints(target1, target2));
    m_target = target2;
//    m_target.header.frame_id = "map";

    PublishTrajectory();
    m_moved = false;
    return (res.success = m_trajectory.points.size() > 0);
}

bool PTPtrajectory::TargetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res)
{
    m_target = req.coords;
//    m_target.header.frame_id = "map";

    UpdateTrajectory(ComputeWaypoints(m_position, m_target));
    PublishTrajectory();

    return (res.success = m_trajectory.points.size() > 0);
}

const sgtdv_msgs::Point2DArr::Ptr PTPtrajectory::ComputeWaypoints(const sgtdv_msgs::Point2D &start, const sgtdv_msgs::Point2D &target) const
{
    const auto targetDistance = static_cast<float>(std::sqrt(
        std::pow(start.x - target.x,2) + std::pow(start.y - target.y,2)));
    
    const auto headingToTarget = std::atan2((target.y - start.y),
                                            (target.x - start.x));
    const auto cosinus = cos(headingToTarget);
    const auto sinus = sin(headingToTarget);

    const auto numOfWaypoints = static_cast<unsigned short>(targetDistance / WAYPOINT_DISTANCE);

    sgtdv_msgs::Point2DArr::Ptr waypoints(new sgtdv_msgs::Point2DArr);
    waypoints->points.reserve(numOfWaypoints);

    for (int i = 0; i < numOfWaypoints; i++)
    {
        {
            sgtdv_msgs::Point2D waypoint;
            waypoint.x = start.x + (i+1) * WAYPOINT_DISTANCE * cosinus;
            waypoint.y = start.y + (i+1) * WAYPOINT_DISTANCE * sinus;
//            waypoint.header.frame_id = "map";
            waypoints->points.emplace_back(waypoint);
        }
    }

    if (numOfWaypoints * WAYPOINT_DISTANCE < targetDistance)
    {
        waypoints->points.push_back(target);
    }

    return waypoints;
}

void PTPtrajectory::UpdateTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &waypoints)
{
    m_trajectory.points.reserve(m_trajectory.points.size() + waypoints->points.size());
    for (auto &i : waypoints->points)
    {
        m_trajectory.points.emplace_back(i);
    }
}

void PTPtrajectory::PublishTrajectory()
{
    m_trajectoryPub.publish(m_trajectory);
    
    if (!ros::service::call("pathTracking/start", m_srvMsg))
    {
        ROS_ERROR("Service \"pathTracking/start\" failed");
    }
}
