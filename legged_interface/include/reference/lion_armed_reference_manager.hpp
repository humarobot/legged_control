
#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>
#include "ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_legged_robot/gait/GaitSchedule.h"
#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"
#include "arm_trajectory_planner.hpp"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class LionArmedReferenceManager : public SwitchedModelReferenceManager {
 public:
  LionArmedReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~LionArmedReferenceManager() override = default;
  vector3_t getReferencePosition(scalar_t time) const;

 private:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  ArmTrajectoryPlanner armTrajectoryPlanner_{0.0};
};

}  // namespace legged_robot
}  // namespace ocs2
