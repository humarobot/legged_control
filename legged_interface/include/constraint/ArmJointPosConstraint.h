
#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

class ArmJointPosConstraint final : public StateConstraint {
 public:
  /*
   * Constructor
   * @param [in] contactPointIndex : The 6 DoF contact index.
   * @param [in] info : The centroidal model information.
   */
  ArmJointPosConstraint(vector3_t);

  ~ArmJointPosConstraint() override = default;
  ArmJointPosConstraint* clone() const override { return new ArmJointPosConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 3; }
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  ArmJointPosConstraint(const ArmJointPosConstraint& other) = default;
  vector3_t arm_joint_pos_desired_;

//   const size_t contactPointIndex_;
//   const CentroidalModelInfo info_;
};

}  // namespace legged_robot
}  // namespace ocs2
