
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
{
}

void LionArmedReferenceManager::subscribe(ros::NodeHandle& nodeHandle)
{
  auto pointCallback = [this](const geometry_msgs::Point::ConstPtr& msg) {
    new_point_ = vector3_t(msg->x, msg->y, msg->z);
  };
  pointSubscriber_ =
      nodeHandle.subscribe<geometry_msgs::Point>("path_point", 1, pointCallback);
}

void LionArmedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                 TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule)
{
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  const scalar_t terrainHeight = 0.02;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
  armTrajectoryPlanner_.update(new_point_,0.2,0.2,initTime);
}

vector3_t LionArmedReferenceManager::getReferencePosition(scalar_t time) const
{
  return armTrajectoryPlanner_.getLinearPositionConstraint(time);
}

}  // namespace legged_robot
}  // namespace ocs2
