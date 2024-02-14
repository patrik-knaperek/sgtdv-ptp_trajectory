/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/ptp_trajectory.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "ptp_trajectory");
  ros::NodeHandle handle;

  PTPtrajectory trajectory_obj(handle);

  ros::spin();

  return 0;
}
