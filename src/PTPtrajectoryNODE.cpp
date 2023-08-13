/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/PTPtrajectory.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "PTPtrajectory");
    ros::NodeHandle handle;

    ros::Publisher trajectoryPub = handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1);
    PTPtrajectory trajectoryObj(trajectoryPub);

    ros::Subscriber poseSub = handle.subscribe<sgtdv_msgs::CarPose>("pose_estimate", 1, &PTPtrajectory::PoseCallback, &trajectoryObj);
    
    ros::ServiceServer rectangleSrv = handle.advertiseService("PTPtrajectory/go_rectangle", &PTPtrajectory::RectangleCallback, &trajectoryObj);
    ros::ServiceServer targetSrv = handle.advertiseService("PTPtrajectory/set_target", &PTPtrajectory::TargetCallback, &trajectoryObj);
    
    ros::spin();

    return 0;
}