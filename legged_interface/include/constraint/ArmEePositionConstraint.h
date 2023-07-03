#pragma once

#include <memory>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>
// #include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "reference/lion_armed_reference_manager.hpp"
#include "reference/arm_trajectory_planner.hpp"

namespace ocs2 {
namespace legged_robot {

class ArmEePositionConstraint final : public StateConstraint {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<scalar_t>;

  ArmEePositionConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const LionArmedReferenceManager& referenceManager);
  ~ArmEePositionConstraint() override = default;
  ArmEePositionConstraint* clone() const override { return new ArmEePositionConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_); }

  size_t getNumConstraints(scalar_t time) const override;
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;

 private:
  ArmEePositionConstraint(const ArmEePositionConstraint& other) = default;
  std::pair<vector_t, quaternion_t> interpolateEndEffectorPose(scalar_t time) const;

  /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
  PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  vector3_t eeDesiredPosition_;
  quaternion_t eeDesiredOrientation_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const LionArmedReferenceManager* referenceManagerPtr_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
