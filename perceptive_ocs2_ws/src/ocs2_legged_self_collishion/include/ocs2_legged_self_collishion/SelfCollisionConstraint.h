/*
 * @Author: NaCl
 * @Date: 2024-04-19 15:46:38
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-19 17:31:44
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_self_collishion/include/ocs2_legged_self_collishion/SelfCollisionConstraint.h
 * @Description:
 *
 */

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_self_collision/SelfCollision.h>

#include "ocs2_unitree_models/QuadrupedPinocchioMapping.h"

#include "ocs2_self_collision/PinocchioGeometryInterface.h"
#include <ocs2_switched_model_interface/core/Rotations.h>



namespace unitree
{

  using namespace ocs2;
  using namespace switched_model;

  /**
   *  This class provides a variant of the Self-collision constraints, which allows for caching. Therefore It is the user's
   *  responsibility to call the required updates on the PinocchioInterface in pre-computation requests.
   */
  class SelfCollisionConstraint : public StateConstraint
  {
  public:
    /**
     * Constructor
     *
     * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
     * @param [in] pinocchioGeometryInterface: Pinocchio geometry interface of the robot model.
     * @param [in] minimumDistance: The minimum allowed distance between collision pairs.
     */
    // SelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t> &mapping, PinocchioGeometryInterface pinocchioGeometryInterface,
    //                         scalar_t minimumDistance);

    SelfCollisionConstraint(const FrameDeclaration &frameDeclaration, PinocchioInterface pinocchioInterface,
                            PinocchioGeometryInterface pinocchioGeometryInterface,
                            scalar_t minimumDistance);

    ~SelfCollisionConstraint() override = default;

    size_t getNumConstraints(scalar_t time) const;

    /** Get the self collision distance values
     *
     * @note Requires pinocchio::forwardKinematics().
     */
    vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComputation) const;

    /** Get the self collision distance approximation
     *
     * @note Requires pinocchio::forwardKinematics(),
     *                pinocchio::updateGlobalPlacements(),
     *                pinocchio::computeJointJacobians().
     * @note In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
     * you should also call tham as well.
     */
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                             const PreComputation &preComputation) const;

    vector_t getPinnochioConfiguration(
        const switched_model::base_coordinate_t &basePose,
        const switched_model::joint_coordinate_t &jointPositions) const
    {
      const auto &model = pinocchioInterface_.getModel();

      vector_t configuration(model.nq);
      // basePost
      configuration.head<3>() = switched_model::getPositionInOrigin(basePose);
      // baseQuad
      const Eigen::Quaternion<scalar_t> baseQuat = switched_model::quaternionBaseToOrigin<scalar_t>(switched_model::getOrientation(basePose));
      configuration.segment<4>(3) = baseQuat.coeffs();
      // JointsPos
      configuration.segment<switched_model::JOINT_COORDINATE_SIZE>(7) = pinocchioMapping_.getPinocchioJointVector(jointPositions);
      return configuration;
    }

  private:
    unitree::QuadrupedPinocchioMapping pinocchioMapping_;

  protected:
    mutable PinocchioInterface pinocchioInterface_;

    SelfCollisionConstraint(const SelfCollisionConstraint &rhs);

    SelfCollision selfCollision_;
    std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
  };

} // namespace ocs2
