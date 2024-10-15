/*
 * @Author: NaCl
 * @Date: 2024-04-16 16:07:43
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-18 19:07:55
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_self_collishion/include/ocs2_legged_self_collishion/ocs2_legged_self_collishion.h
 * @Description:
 *
 */

#pragma once

#include <memory>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>
#include "ocs2_unitree_models/QuadrupedPinocchioMapping.h"

#include "ocs2_self_collision/PinocchioGeometryInterface.h"

// #include <ocs2_self_collision/SelfCollisionCppAd.h>
#include <ocs2_self_collision/SelfCollision.h>

namespace unitree
{

  using namespace ocs2;
  using namespace switched_model;

  /**
   * Implements the collision avoidance penalty function for all collision spheres given by the precomputation.
   * Uses a Gauss-Newton approximation to generate a positive semi-definite cost Hessian w.r.t. state.
   */
  class SelfCollisionAvoidanceCost final : public ocs2::StateCost
  {
  public:

    SelfCollisionAvoidanceCost(const FrameDeclaration &frameDeclaration, PinocchioInterface pinocchioInterface,
                               ocs2::RelaxedBarrierPenalty::Config settings,
                               PinocchioGeometryInterface pinocchioGeometryInterface,
                               scalar_t minimumDistance,
                               const std::string &modelName,
                               const std::string &modelFolder = "/tmp/ocs2",
                               bool recompileLibraries = true,
                               bool verbose = true);

    SelfCollisionAvoidanceCost *clone() const override { return new SelfCollisionAvoidanceCost(*this); }

    scalar_t getValue(scalar_t time, const vector_t &state, const ocs2::TargetTrajectories &targetTrajectories,
                      const ocs2::PreComputation &preComp) const override;

    ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t &state,
                                                                   const ocs2::TargetTrajectories &targetTrajectories,
                                                                   const ocs2::PreComputation &preComp) const override;

  private:
    std::unique_ptr<ocs2::RelaxedBarrierPenalty> penalty_;
    unitree::QuadrupedPinocchioMapping pinocchioMapping_;

  protected:
    mutable PinocchioInterface pinocchioInterface_;
    // SelfCollisionCppAd selfCollision_;
    SelfCollision selfCollision_;

    SelfCollisionAvoidanceCost(const SelfCollisionAvoidanceCost &rhs);
  };

} // namespace switched_model
