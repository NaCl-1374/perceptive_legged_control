//
// Created by qiayuan on 23-1-1.
//

#pragma once

#include <legged_interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
// #include <ocs2_core/dynamics/ControlledSystemBase.h>

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class FootPlacementConstraintCBF final : public StateInputConstraint
{
public:
  struct Parameter
  {
    matrix_t a;
    vector_t b;
  };

  FootPlacementConstraintCBF(const SwitchedModelReferenceManager& referenceManager,
                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                             size_t numVertices,scalar_t cbfLambda);

  ~FootPlacementConstraintCBF() override = default;
  FootPlacementConstraintCBF* clone() const override
  {
    return new FootPlacementConstraintCBF(*this);
  }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t /*time*/) const override
  {
    return numVertices_;
  }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

private:
  FootPlacementConstraintCBF(const FootPlacementConstraintCBF& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  // std::unique_ptr<ControlledSystemBase> systemDynamicsPtr_;

  const size_t contactPointIndex_;
  const size_t numVertices_;

  scalar_t lamda_;
};

}  // namespace legged
