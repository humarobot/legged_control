
#include "BaseWrenchConstraint.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BaseWrenchConstraint::BaseWrenchConstraint(const BaseWrenchReferenceManager& referenceManager,
                                         CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t BaseWrenchConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  vector_t wrench_b(6),wrench_w(6);
  vector3_t rpy;
  rpy = state.segment<3>(9);
  wrench_b = referenceManagerPtr_->getWrench();
  wrench_w.head(3) = getRotationMatrixFromZyxEulerAngles(rpy)*wrench_b.head(3);
  wrench_w.tail(3) = getRotationMatrixFromZyxEulerAngles(rpy)*wrench_b.tail(3);
  vector_t value = input.segment(3 * 4, 6)-wrench_w;
  return value;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation BaseWrenchConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(6, state.size());
  approx.dfdu = matrix_t::Zero(6, input.size());
  approx.dfdu.middleCols<6>(3 * 4).diagonal() = vector_t::Ones(6);
  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
