
#include "constraint/ArmJointPosConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ArmJointPosConstraint::ArmJointPosConstraint(vector3_t arm_joint_pos_desired) : 
StateConstraint(ConstraintOrder::Linear) ,arm_joint_pos_desired_(arm_joint_pos_desired){}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ArmJointPosConstraint::isActive(scalar_t time) const { return true; }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ArmJointPosConstraint::getValue(scalar_t time, const vector_t& state,const PreComputation& preComp) const {
    vector_t arm_joint_pos = state.tail(3);
    arm_joint_pos -= arm_joint_pos_desired_;
    return arm_joint_pos;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ArmJointPosConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, preComp);
  approx.dfdx = matrix_t::Zero(3, state.size());
  approx.dfdx.rightCols(3).diagonal() = vector_t::Ones(3);
  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
