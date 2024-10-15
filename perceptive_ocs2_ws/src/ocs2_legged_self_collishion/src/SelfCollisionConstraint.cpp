/*
 * @Author: NaCl
 * @Date: 2024-04-19 15:46:53
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-19 18:12:46
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_self_collishion/src/SelfCollisionConstraint.cpp
 * @Description:
 *
 */
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_legged_self_collishion/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollision.h>

namespace unitree
{

  using namespace ocs2;
  using namespace switched_model;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/

  SelfCollisionConstraint::SelfCollisionConstraint(const FrameDeclaration &frameDeclaration, PinocchioInterface pinocchioInterface,
                                                   PinocchioGeometryInterface pinocchioGeometryInterface,
                                                   scalar_t minimumDistance)
      : StateConstraint(ConstraintOrder::Linear),
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
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SelfCollisionConstraint::SelfCollisionConstraint(const SelfCollisionConstraint &rhs)
      : StateConstraint(rhs), pinocchioInterface_(rhs.pinocchioInterface_),
        pinocchioMapping_(rhs.pinocchioMapping_),
        selfCollision_(rhs.selfCollision_) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t SelfCollisionConstraint::getNumConstraints(scalar_t time) const
  {
    return selfCollision_.getNumCollisionPairs();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t SelfCollisionConstraint::getValue(scalar_t time, const vector_t &state, const PreComputation &preComputation) const
  {
    // const auto q = getPinnochioConfiguration(getBasePose(state), getJointPositions(state));

    const auto q = pinocchioMapping_.getPinocchioJointVector(joint_coordinate_t(state.tail(JOINT_COORDINATE_SIZE)));
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, q);

    // auto selfDistance = selfCollision_.getValue(pinocchioInterface_);

    // const auto &pinocchioInterface = getPinocchioInterface(preComputation);
    return selfCollision_.getValue(pinocchioInterface_) ;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation SelfCollisionConstraint::getLinearApproximation(scalar_t time, const vector_t &state,
                                                                                    const PreComputation &preComputation) const
  {
    // const auto q = getPinnochioConfiguration(getBasePose(state), getJointPositions(state));

    const auto q = pinocchioMapping_.getPinocchioJointVector(joint_coordinate_t(state.tail(JOINT_COORDINATE_SIZE)));
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    VectorFunctionLinearApproximation constraint;
    matrix_t dfdq, dfdv;
    std::tie(constraint.f, dfdq) = selfCollision_.getLinearApproximation(pinocchioInterface_);
    // std::cout << "dfdq size " << dfdq.rows() << " " << dfdq.cols() << "\n"
    //           << dfdq << "\n";

    matrix_t dfdqOcs2;
    dfdqOcs2.setZero(dfdq.rows(), dfdq.cols());

    dfdqOcs2 = dfdq;

    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) // pinocchio的足端顺序排列雅可比 ocs2的是LF RF LH RH pinocchoi 的是  LF LH RF RH
    {
      size_t ocs2_i = pinocchioMapping_.getOcs2FootIndex(i); // pinocchio到ocs2顺序
      dfdqOcs2.block(0, 3 * ocs2_i, dfdq.rows(), 3) = dfdq.block(0, 3 * i, dfdq.rows(), 3);
    }
    matrix_t dqdx; // 就是dqdx
    dqdx.setZero(19, STATE_DIM);
    dqdx.rightCols(18).setIdentity();
    constraint.dfdx = dfdqOcs2 * dqdx;


    // VectorFunctionLinearApproximation constraint1;
    // constraint1.f.setZero(constraint.f.size());
    // constraint1.dfdx.setZero(8, 24);

    return constraint;
  }

} // namespace ocs2
