/*
 * @Author: NaCl
 * @Date: 2024-04-16 16:07:47
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-19 15:33:41
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_self_collishion/src/ocs2_legged_self_collishion.cpp
 * @Description:
 *
 */
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_legged_self_collishion/ocs2_legged_self_collishion.h"
#include <ocs2_switched_model_interface/cost/LinearStateInequalitySoftconstraint.h>

// #include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_self_collision/SelfCollision.h>

namespace unitree
{

  SelfCollisionAvoidanceCost::SelfCollisionAvoidanceCost(const FrameDeclaration &frameDeclaration, PinocchioInterface pinocchioInterface,
                                                         ocs2::RelaxedBarrierPenalty::Config settings,
                                                         PinocchioGeometryInterface pinocchioGeometryInterface,
                                                         scalar_t minimumDistance,
                                                         const std::string &modelName,
                                                         const std::string &modelFolder,
                                                         bool recompileLibraries,
                                                         bool verbose)
      : penalty_(std::make_unique<RelaxedBarrierPenalty>(settings)),
        pinocchioMapping_(frameDeclaration, pinocchioInterface),
        pinocchioInterface_(std::move(pinocchioInterface)),
        selfCollision_(std::move(pinocchioGeometryInterface), minimumDistance)
  {
    std::cerr << "[SelfCollision ] \n";
    std::cerr << "pinocchioInterface_ nq " << pinocchioInterface_.getModel().nq << "\n";
    std::cerr << "pinocchioInterface_ nv " << pinocchioInterface_.getModel().nv << "\n";
    std::cerr << "pinocchioInterface_ njoints " << pinocchioInterface_.getModel().njoints << "\n";
    std::cerr << "pinocchioInterface_ JointNames \n";

    for (const std::string &str : pinocchioInterface_.getModel().names)
    {
      std::cout << str << "\t";
    }
    std::cerr << " \n";
    for (int i = 0; i < 4; i++)
    {
      std::cout << "pinocchio id " << i << " -> "
                << "ocs2 id :" << pinocchioMapping_.getOcs2FootIndex(i) << "\n";
    }

    for (int i = 0; i < 4; i++)
    {
      std::cout << "ocs2 id " << i << " -> "
                << "pinocchio id :" << pinocchioMapping_.getPinocchioFootIndex(i) << "\n";
    }
    // const size_t numCollisionPairs = geometryInterfacePtr_->getNumCollisionPairs();
    // const size_t numCollisionPairs = pinocchioGeometryInterface.getNumCollisionPairs();
    // std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
  }

  SelfCollisionAvoidanceCost::SelfCollisionAvoidanceCost(const SelfCollisionAvoidanceCost &rhs)
      : penalty_(rhs.penalty_->clone()),
        pinocchioInterface_(rhs.pinocchioInterface_),
        pinocchioMapping_(rhs.pinocchioMapping_),
        selfCollision_(rhs.selfCollision_)
  {
  }

  scalar_t SelfCollisionAvoidanceCost::getValue(scalar_t time, const vector_t &state, const ocs2::TargetTrajectories &targetTrajectories,
                                                const ocs2::PreComputation &preComp) const
  {

    const auto q = pinocchioMapping_.getPinocchioJointVector(joint_coordinate_t(state.tail(JOINT_COORDINATE_SIZE)));
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, q);

    auto selfDistance = selfCollision_.getValue(pinocchioInterface_);
    // std::cerr << " state  " << state.transpose() << "\n";

    // std::cerr << " q  " << q.transpose() << "\n";

    // std::cerr << " selfDistance " << selfDistance.transpose() << "\n";
    /*selfDistance
    0.0532516  0.059404  0.300077  0.302467 0.0408131 0.0469136  0.376168  0.391902*/

    scalar_t cost(0.0);

    LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
    linearStateInequalitySoftConstraint.penalty = penalty_.get();
    // linearStateInequalitySoftConstraint.A = Leave empty
    linearStateInequalitySoftConstraint.h = selfDistance;

    // cost +=
    // std::cerr << "costs list"
    //           << "\n";

    for (int j = 0; j < selfDistance.size(); ++j)
    { // Loop through all faces of the constraint
      auto temp = penalty_.get()->getValue(0.0, selfDistance(j));
      // std::cerr << temp << " ";
      cost += temp;
    }

    // std::cerr << "\n cost sum" << cost << "\n";
    // auto temp = switched_model::getValue(linearStateInequalitySoftConstraint, q); // 这里的q没用 占位
    // cost += temp;
    return cost;
  }

  ScalarFunctionQuadraticApproximation SelfCollisionAvoidanceCost::getQuadraticApproximation(scalar_t time, const vector_t &state,
                                                                                             const ocs2::TargetTrajectories &targetTrajectories,
                                                                                             const ocs2::PreComputation &preComp) const
  {
    // const auto &switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);

    ScalarFunctionQuadraticApproximation cost; // ocs2顺序
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(STATE_DIM);
    cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);

    const auto q = pinocchioMapping_.getPinocchioJointVector(joint_coordinate_t(state.tail(JOINT_COORDINATE_SIZE))); // pinocchio顺序
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, q);

    matrix_t dfdq, funVal;
    std::tie(funVal, dfdq) = selfCollision_.getLinearApproximation(pinocchioInterface_); // dhdq //pinocchio顺序

    matrix_t dfdqOcs2;
    dfdqOcs2.setZero(dfdq.rows(), dfdq.cols());

    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) // pinocchio的足端顺序排列雅可比 ocs2的是LF RF LH RH pinocchoi 的是  LF LH RF RH
    {
      size_t ocs2_i = pinocchioMapping_.getOcs2FootIndex(i); // pinocchio到ocs2顺序
      dfdqOcs2.block(0, 3 * ocs2_i, dfdq.rows(), 3) = dfdq.block(0, 3 * i, dfdq.rows(), 3);
    }

    LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
    linearStateInequalitySoftConstraint.penalty = penalty_.get();
    linearStateInequalitySoftConstraint.A = dfdqOcs2;
    linearStateInequalitySoftConstraint.h = funVal - dfdq * q;

    matrix_t dqdx; // 就是dqdx
    dqdx.setZero(JOINT_COORDINATE_SIZE, STATE_DIM);
    dqdx.rightCols(12).setIdentity();

    ScalarFunctionQuadraticApproximation targetcost;

    const auto h = funVal - dfdq * q;

    targetcost.f = 0.0;
    for (int j = 0; j < h.size(); ++j)
    { // Loop through all faces of the constraint
      targetcost.f += penalty_.get()->getValue(0.0, h(j));
    }

    const auto penaltyDerivatives = h.unaryExpr([&](scalar_t hi)
                                                { return penalty_.get()->getDerivative(0.0, hi); });
    const vector_t taskSpaceDerivative = dfdqOcs2.transpose() * penaltyDerivatives;
    targetcost.dfdx.noalias() = dqdx.transpose() * taskSpaceDerivative;

    const auto penaltySecondDerivatives = h.unaryExpr([&](scalar_t hi)
                                                      { return penalty_.get()->getSecondDerivative(0.0, hi); });
    const matrix_t scaledConstraint = penaltySecondDerivatives.asDiagonal() * dfdqOcs2;
    const matrix_t taskSpaceSecondDerivative = dfdqOcs2.transpose() * scaledConstraint;
    const matrix_t scaledJacobian = taskSpaceSecondDerivative * dqdx;
    targetcost.dfdxx.noalias() = dqdx.transpose() * scaledJacobian;

    cost.f += targetcost.f;
    cost.dfdx += targetcost.dfdx;
    // cost.dfdxx += targetcost.dfdxx;
    return cost;
  }
} // namespace switched_model
