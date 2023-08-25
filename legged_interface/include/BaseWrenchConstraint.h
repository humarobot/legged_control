#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "BaseWrenchReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

class BaseWrenchConstraint final : public StateInputConstraint {
 public:
  /*
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager.
   * @param [in] info : The centroidal model information.
   */
  BaseWrenchConstraint(const BaseWrenchReferenceManager& referenceManager, CentroidalModelInfo info);

  ~BaseWrenchConstraint() override = default;
  BaseWrenchConstraint* clone() const override { return new BaseWrenchConstraint(*this); }

  bool isActive(scalar_t time) const override{ return true;}
  size_t getNumConstraints(scalar_t time) const override { return 6; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  BaseWrenchConstraint(const BaseWrenchConstraint& other) = default;

  const BaseWrenchReferenceManager* referenceManagerPtr_;
  const CentroidalModelInfo info_;
};

}  // namespace legged_robot
}  // namespace ocs2