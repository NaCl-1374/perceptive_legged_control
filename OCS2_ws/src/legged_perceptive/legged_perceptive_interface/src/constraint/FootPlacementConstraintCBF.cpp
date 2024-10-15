//
// Created by qiayuan on 23-1-1.
//

#include "legged_perceptive_interface/constraint/FootPlacementConstraintCBF.h"
#include "legged_perceptive_interface/PerceptiveLeggedPrecomputation.h"
#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"

namespace legged
{
FootPlacementConstraintCBF::FootPlacementConstraintCBF(const SwitchedModelReferenceManager& referenceManager,
                                                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                       size_t contactPointIndex, size_t numVertices, scalar_t cbfLambda)
  : StateInputConstraint(ConstraintOrder::Linear)
  , referenceManagerPtr_(&referenceManager)
  , endEffectorKinematicsPtr_(endEffectorKinematics.clone())
  , contactPointIndex_(contactPointIndex)
  , numVertices_(numVertices)
  // , systemDynamicsPtr_(systemDynamics.clone())
  , lamda_(cbfLambda)
{
}

FootPlacementConstraintCBF::FootPlacementConstraintCBF(const FootPlacementConstraintCBF& rhs)
  : StateInputConstraint(ConstraintOrder::Linear)
  , referenceManagerPtr_(rhs.referenceManagerPtr_)
  , endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone())
  , contactPointIndex_(rhs.contactPointIndex_)
  , numVertices_(rhs.numVertices_)
  // , systemDynamicsPtr_(rhs.systemDynamicsPtr_->clone())
  , lamda_(rhs.lamda_)

{
}

bool FootPlacementConstraintCBF::isActive(scalar_t time) const
{
  return dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_)
      .getFootPlacementFlags(time)[contactPointIndex_];
}

vector_t FootPlacementConstraintCBF::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                              const PreComputation& preComp) const
{
  const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  // std::cout << param.a << std::endl;
  const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();
  vector_t dot_hs = param.a * velocity;
  vector_t hs = param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
  return dot_hs + hs * lamda_;
}

VectorFunctionLinearApproximation FootPlacementConstraintCBF::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const
{
  const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];

  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

  const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
  const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();

  vector_t dot_hs = param.a * velocity;
  vector_t hs = param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;

  VectorFunctionLinearApproximation approx =
      VectorFunctionLinearApproximation::Zero(numVertices_, state.size(), input.size());

  approx.f = dot_hs + hs * lamda_;
  approx.dfdx = param.a * velocityApprox.dfdx + param.a * positionApprox.dfdx * lamda_;
  approx.dfdu += param.a * velocityApprox.dfdu;

  return approx;
}

// vector_t FootPlacementConstraintCBF::getValue(scalar_t time, const vector_t& state, const vector_t& input,
//                                               const PreComputation& preComp) const
// {
//   const auto param =
//   cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_]; return param.a *
//   endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
// }

// VectorFunctionLinearApproximation FootPlacementConstraintCBF::getLinearApproximation(
//     scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const
// {
//   VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numVertices_, state.size(),
//   input.size()); const auto param =
//   cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];

//   const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
//   approx.f = param.a * positionApprox.f + param.b;
//   approx.dfdx = param.a * positionApprox.dfdx;
//   return approx;
// }  // namespace legged

}  // namespace legged
