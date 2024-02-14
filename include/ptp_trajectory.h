/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <iostream>
#include <cmath>
#include <vector>

/* ROS */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

/* SGT */
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <ptp_trajectory/GoRectangle.h>
#include <ptp_trajectory/SetTarget.h>

class PTPtrajectory
{
public:
  static constexpr float WAYPOINT_DISTANCE = 0.5;
  static constexpr float MIN_DISTANCE = 0.2;

public:
  PTPtrajectory(ros::NodeHandle& nh);
  ~PTPtrajectory() = default;

  void poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
  bool rectangleCallback(ptp_trajectory::GoRectangle::Request& req, ptp_trajectory::GoRectangle::Response& res);
  bool targetCallback(ptp_trajectory::SetTarget::Request& req, ptp_trajectory::SetTarget::Response& res);

private:
  const sgtdv_msgs::Point2DArr::Ptr computeWaypoints(const sgtdv_msgs::Point2D &start, const sgtdv_msgs::Point2D &target) const;
  void updateTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &waypoints);
  void publishTrajectory();

  ros::Publisher trajectory_pub_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer rectangle_srv_;
  ros::ServiceServer target_srv_;

  sgtdv_msgs::Point2D target_;
  sgtdv_msgs::Point2DArr trajectory_;
  sgtdv_msgs::Point2D position_;
  std_srvs::Empty srv_msg_;
  bool moved_ = false;
};
