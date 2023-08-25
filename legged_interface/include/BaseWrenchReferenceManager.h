
#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_legged_robot/gait/GaitSchedule.h"
#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"

#include <ros/ros.h>
#include "geometry_msgs/Wrench.h"


namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class BaseWrenchReferenceManager {
 public:
  BaseWrenchReferenceManager();
  ~BaseWrenchReferenceManager() = default;

  void subscribe(ros::NodeHandle& nodeHandle);
  vector_t getWrench() const { return wrench_; }
 private:
  ::ros::Subscriber wrenchSubscriber_;
  vector_t wrench_;

};

}  // namespace legged_robot
}  // namespace ocs2