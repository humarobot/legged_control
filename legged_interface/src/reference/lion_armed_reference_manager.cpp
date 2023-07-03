
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LionArmedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                 TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule)
{
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  const scalar_t terrainHeight = 0.02;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

vector3_t LionArmedReferenceManager::getReferencePosition(scalar_t time) const{
  return armTrajectoryPlanner_.getLinearPositionConstraint(time);
}

}  // namespace legged_robot
}  // namespace ocs2
