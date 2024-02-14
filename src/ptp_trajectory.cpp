/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/ptp_trajectory.h"

PTPtrajectory::PTPtrajectory(ros::NodeHandle& nh) :
   /* ROS interface init */
  trajectory_pub_(nh.advertise<sgtdv_msgs::Point2DArr>("path_planning/trajectory", 1)),
  
  pose_sub_(nh.subscribe<sgtdv_msgs::CarPose>("odometry/pose", 1, &PTPtrajectory::poseCallback, this)),
  
  rectangle_srv_(nh.advertiseService("ptp_trajectory/go_rectangle", &PTPtrajectory::rectangleCallback, this)),
  target_srv_(nh.advertiseService("ptp_trajectory/set_target", &PTPtrajectory::targetCallback, this))
{
}

void PTPtrajectory::poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
  position_ = msg->position;

  const auto target_dist = std::sqrt(std::pow(target_.x - position_.x, 2) + std::pow(target_.y - position_.y, 2));
  if(!moved_)
  {
    if(target_dist > 4.0 * MIN_DISTANCE)
        moved_ = true;
  }
  else if(target_dist < MIN_DISTANCE)
  {
    if(!ros::service::call("path_tracking/stop", srv_msg_))
    {
      ROS_ERROR("Service \"path_tracking/stop\" failed");
    }
    trajectory_.points.clear();
  }
}

bool PTPtrajectory::rectangleCallback(ptp_trajectory::GoRectangle::Request& req, ptp_trajectory::GoRectangle::Response& res)
{
  sgtdv_msgs::Point2D target1;
  target1.x = position_.x + req.a / 2.;
  target1.y = position_.y;
  updateTrajectory(computeWaypoints(position_, target1));

  sgtdv_msgs::Point2D target2;
  target2.x = target1.x;
  target2.y = position_.y + req.b * (req.right ? -1 : 1);
  updateTrajectory(computeWaypoints(target1, target2));
  
  target1 = target2;
  target2.x -= req.a;
  updateTrajectory(computeWaypoints(target1, target2));

  target1 = target2;
  target2.y = position_.y;
  updateTrajectory(computeWaypoints(target1, target2));

  target1 = target2;
  target2.x = position_.x - MIN_DISTANCE;
  updateTrajectory(computeWaypoints(target1, target2));
  target_ = target2;

  publishTrajectory();
  moved_ = false;
  return (res.success = trajectory_.points.size() > 0);
}

bool PTPtrajectory::targetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res)
{
  target_ = req.coords;

  updateTrajectory(computeWaypoints(position_, target_));
  publishTrajectory();

  return (res.success = trajectory_.points.size() > 0);
}

const sgtdv_msgs::Point2DArr::Ptr PTPtrajectory::computeWaypoints(const sgtdv_msgs::Point2D &start, const sgtdv_msgs::Point2D &target) const
{
  const auto target_distance = static_cast<float>(
    std::sqrt(
      std::pow(start.x - target.x,2) + std::pow(start.y - target.y,2)
    ));
  
  const auto heading_to_target = std::atan2((target.y - start.y),
                                          (target.x - start.x));
  const auto cosinus = cos(heading_to_target);
  const auto sinus = sin(heading_to_target);

  const auto num_of_waypoints = static_cast<unsigned short>(target_distance / WAYPOINT_DISTANCE);

  sgtdv_msgs::Point2DArr::Ptr waypoints(new sgtdv_msgs::Point2DArr);
  waypoints->points.reserve(num_of_waypoints);

  for(int i = 0; i < num_of_waypoints; i++)
  {
    {
      sgtdv_msgs::Point2D waypoint;
      waypoint.x = start.x + (i+1) * WAYPOINT_DISTANCE * cosinus;
      waypoint.y = start.y + (i+1) * WAYPOINT_DISTANCE * sinus;
      waypoints->points.emplace_back(waypoint);
    }
  }

  if(num_of_waypoints * WAYPOINT_DISTANCE < target_distance)
  {
    waypoints->points.push_back(target);
  }

  return waypoints;
}

void PTPtrajectory::updateTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &waypoints)
{
  trajectory_.points.reserve(trajectory_.points.size() + waypoints->points.size());
  for(const auto &i : waypoints->points)
  {
    trajectory_.points.emplace_back(i);
  }
}

void PTPtrajectory::publishTrajectory()
{
  trajectory_pub_.publish(trajectory_);
  
  if(!ros::service::call("path_tracking/start", srv_msg_))
  {
    ROS_ERROR("Service \"path_tracking/start\" failed");
  }
}
