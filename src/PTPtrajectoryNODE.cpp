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
    ros::Publisher stopPub = handle.advertise<std_msgs::Empty>("stop",1);
    ros::Publisher startPub = handle.advertise<std_msgs::Empty>("start",1);
    PTPtrajectory trajectoryObj(trajectoryPub, stopPub, startPub);

    ros::Subscriber poseSub = handle.subscribe<sgtdv_msgs::CarPose>("pose_estimate", 1, &PTPtrajectory::PoseCallback, &trajectoryObj);
    
    ros::ServiceServer rectangleSrv = handle.advertiseService("ptp_trajectory/go_rectangle", &PTPtrajectory::RectangleCallback, &trajectoryObj);
    ros::ServiceServer targetSrv = handle.advertiseService("ptp_trajectory/set_target", &PTPtrajectory::TargetCallback, &trajectoryObj);
    trajectoryObj.SetSrvServers(rectangleSrv, targetSrv);


    ros::spin();

    return 0;
}