#include "constraint/ArmEePositionConstraint.h"
#include <ocs2_core/misc/LinearInterpolation.h>
#include "ocs2_legged_robot/LeggedRobotPreComputation.h"


namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ArmEePositionConstraint::ArmEePositionConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()) {
  if (endEffectorKinematics.getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ArmEePositionConstraint::getNumConstraints(scalar_t time) const { return 3; }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ArmEePositionConstraint::getValue(scalar_t time, const vector_t& state,
                                            const PreComputation& preComputation) const {

  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<legged_robot::LeggedRobotPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }
  vector3_t base_pos = state.segment<3>(6);
  vector3_t base_euler = state.segment<3>(9);
  quaternion_t base_quat = quaternion_t(Eigen::AngleAxisd(base_euler[2], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(base_euler[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(base_euler[0], Eigen::Vector3d::UnitZ()));

  vector3_t relative_pos;
  relative_pos<<0.1, 0.0, 0.3;
  vector3_t pos = base_pos + base_quat * relative_pos;

  vector_t constraint(3);
  constraint = endEffectorKinematicsPtr_->getPosition(state).front() - pos;

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ArmEePositionConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {

  // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<legged_robot::LeggedRobotPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }
  vector3_t base_pos = state.segment<3>(6);
  vector3_t base_euler = state.segment<3>(9);
  quaternion_t base_quat = quaternion_t(Eigen::AngleAxisd(base_euler[2], Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(base_euler[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(base_euler[0], Eigen::Vector3d::UnitZ()));

  vector3_t relative_pos;
  relative_pos<<0.1,0.0,0.3;

  vector3_t pos = base_pos + base_quat * relative_pos;
  
  auto approximation = VectorFunctionLinearApproximation(3, state.rows(), 0);

  const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approximation.f.head<3>() = eePosition.f - pos;
  approximation.dfdx.topRows<3>() = eePosition.dfdx;
  return approximation;
}


}  // namespace legged_robot
}  // namespace ocs2
