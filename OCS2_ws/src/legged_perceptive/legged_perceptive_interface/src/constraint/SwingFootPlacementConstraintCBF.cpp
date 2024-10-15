//
// Created by qiayuan on 23-1-1.
//

#include "legged_perceptive_interface/constraint/SwingFootPlacementConstraintCBF.h"

#include "legged_perceptive_interface/PerceptiveLeggedPrecomputation.h"
#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"

namespace legged {
SwingFootPlacementConstraintCBF::SwingFootPlacementConstraintCBF(
    const SwitchedModelReferenceManager& referenceManager,
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    size_t contactPointIndex, size_t numVertices, scalar_t cbfLambda,
    std::unique_ptr<PenaltyBase> penaltyFunction)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      numVertices_(numVertices)
      // , systemDynamicsPtr_(systemDynamics.clone())
      ,
      lamda_(cbfLambda),
      penaltyPtr_(std::move(penaltyFunction)) {}

SwingFootPlacementConstraintCBF::SwingFootPlacementConstraintCBF(
    const SwingFootPlacementConstraintCBF& rhs)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      numVertices_(rhs.numVertices_)
      // , systemDynamicsPtr_(rhs.systemDynamicsPtr_->clone())
      ,
      lamda_(rhs.lamda_),
      penaltyPtr_(rhs.penaltyPtr_->clone())

{}

bool SwingFootPlacementConstraintCBF::isActive(scalar_t time) const {

  // 摆动相位激活
  return !dynamic_cast<const PerceptiveLeggedReferenceManager&>(
              *referenceManagerPtr_)
              .getContactFlags(time)[contactPointIndex_];
}

vector_t SwingFootPlacementConstraintCBF::getValue(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  const PerceptiveLeggedReferenceManager& temp_reference =
      dynamic_cast<const PerceptiveLeggedReferenceManager&>(
          *referenceManagerPtr_);

  auto SwingPhase = temp_reference.SwingPhasePerLeg(
      time)[contactPointIndex_];  // 0~1 in swing phase -1 in contact phase
                                  // phase=(t-ts)/duration ,dphase/dt=1/duration
  auto duration = 1.0 / SwingPhase.duration;
  auto St = penaltyPtr_->getValue(time, SwingPhase.phase);
  auto dotSt = penaltyPtr_->getDerivative(time, SwingPhase.phase) * duration;

  const auto param = cast<PerceptiveLeggedPrecomputation>(preComp)
                         .getFootPlacementConParameters()[contactPointIndex_];
  // std::cout << param.a << std::endl;
  const auto velocity =
      endEffectorKinematicsPtr_->getVelocity(state, input).front();

  vector_t hs =
      param.a * endEffectorKinematicsPtr_->getPosition(state).front() +
      param.b + St * vector_t::Ones(param.b.size());
  vector_t dot_hs = param.a * velocity + dotSt * vector_t::Ones(param.b.size());

  // return dot_hs + hs * lamda_;
    return  hs;

}

VectorFunctionLinearApproximation
SwingFootPlacementConstraintCBF::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const PreComputation& preComp) const {
  const PerceptiveLeggedReferenceManager& temp_reference =
      dynamic_cast<const PerceptiveLeggedReferenceManager&>(
          *referenceManagerPtr_);

  auto SwingPhase = temp_reference.SwingPhasePerLeg(time)[contactPointIndex_];
  auto duration = 1.0 / SwingPhase.duration;
  auto St = penaltyPtr_->getValue(time, SwingPhase.phase);
  // std::cout<<"st: "<<St<<std::endl;
  auto dotSt = penaltyPtr_->getDerivative(time, SwingPhase.phase) * duration;
  // std::cout<<"dotSt: "<<dotSt<<std::endl;
  const auto param = cast<PerceptiveLeggedPrecomputation>(preComp)
                         .getFootPlacementConParameters()[contactPointIndex_];

  const auto positionApprox =
      endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

  const auto velocityApprox =
      endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input)
          .front();

  vector_t hs =
      param.a * endEffectorKinematicsPtr_->getPosition(state).front() +
      param.b + St * vector_t::Ones(param.b.size());
  vector_t dot_hs = param.a * velocityApprox.f + dotSt * vector_t::Ones(param.b.size());

  VectorFunctionLinearApproximation approx =
      VectorFunctionLinearApproximation::Zero(numVertices_, state.size(),
                                              input.size());

  // approx.f = dot_hs + hs * lamda_;
  // approx.dfdx =
  //     param.a * velocityApprox.dfdx + param.a * positionApprox.dfdx * lamda_;
  // approx.dfdu += param.a * velocityApprox.dfdu;

  approx.f = hs ;
  approx.dfdx =param.a * positionApprox.dfdx;

  return approx;
}

}  // namespace legged
