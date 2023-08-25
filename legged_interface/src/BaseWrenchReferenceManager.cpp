#include "BaseWrenchReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

BaseWrenchReferenceManager::BaseWrenchReferenceManager(){
  wrench_.resize(6);
  wrench_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

void BaseWrenchReferenceManager::subscribe(ros::NodeHandle& nodeHandle){
  auto wrenchCallback = [this](const geometry_msgs::Wrench::ConstPtr& msg) {
    wrench_.resize(6);
    wrench_ << msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z;
    //print wrench
    std::cout << "wrench: " << wrench_.transpose() << std::endl;
  };
  wrenchSubscriber_ = nodeHandle.subscribe<geometry_msgs::Wrench>("/baseWrench", 1, wrenchCallback);
}

}  // namespace legged_robot
}  // namespace ocs2
