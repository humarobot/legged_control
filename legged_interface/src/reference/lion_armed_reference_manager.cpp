
#include "reference/lion_armed_reference_manager.hpp"

namespace ocs2
{
namespace legged_robot
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LionArmedReferenceManager::LionArmedReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                     std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
  : SwitchedModelReferenceManager(gaitSchedulePtr, swingTrajectoryPtr)
  , gaitSchedulePtr_(gaitSchedulePtr)
  , swingTrajectoryPtr_(swingTrajectoryPtr)
  , new_point_(0.4,0.0,0.2)
{
}

void LionArmedReferenceManager::subscribe(ros::NodeHandle& nodeHandle)
{
  auto pointCallback = [this](const geometry_msgs::Point::ConstPtr& msg) {
    new_point_ = vector3_t(msg->x, msg->y, msg->z);
  };
  auto baseGoalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
    base_goal_pos_ = vector3_t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    base_goal_orientation_ = quaternion_t(msg->pose.orientation.w, msg->pose.orientation.z, msg->pose.orientation.y, msg->pose.orientation.x);
  };
  pointSubscriber_ =
      nodeHandle.subscribe<geometry_msgs::Point>("path_point", 1, pointCallback);
  baseGoalSubscriber_ =
      nodeHandle.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, baseGoalCallback);
}

void LionArmedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                 TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule)
{
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  const scalar_t terrainHeight = 0.02;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
  armTrajectoryPlanner_.update(new_point_,2.0,2.0,initTime);
}

vector3_t LionArmedReferenceManager::getReferencePosition(scalar_t time) const
{
  return armTrajectoryPlanner_.getLinearPositionConstraint(time);
}

quaternion_t LionArmedReferenceManager::getReferenceOrientation(scalar_t time) const
{
  // quaternion_t q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  // std::cerr<<q.coeffs().transpose()<<std::endl;
  // std::cerr<<base_goal_orientation_.coeffs().transpose()<<std::endl;
  return base_goal_orientation_;
}

}  // namespace legged_robot
}  // namespace ocs2
