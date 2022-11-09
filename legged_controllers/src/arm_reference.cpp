#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "arm_reference.h"

void ArmReference::subscribe(ros::NodeHandle& nodeHandle){
    auto armReferenceCallback = [this](const geomotry_msgs::PoseConstPtr& msg) {
        sub_flag_ = true;
    };
    armRefSubscriber_ = nodeHandle.subscribe<geomotry_msgs::Pose>(topicPrefix_ + "_EE_pose", 1, armReferenceCallback);

}