#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "arm_reference.h"

void ArmReference::subscribe(ros::NodeHandle& nodeHandle){
    auto armReferenceCallback = [this](const geometry_msgs::PoseConstPtr& msg) {
        sub_flag_ = true;
    };
    ros::NodeHandle nh;

    armRefSubscriber_ = nh.subscribe<geometry_msgs::Pose>(topicPrefix_ + "_EE_pose", 10, armReferenceCallback);

}