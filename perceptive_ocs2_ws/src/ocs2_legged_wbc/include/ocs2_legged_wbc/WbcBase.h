
//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "ocs2_legged_wbc/Task.h"

// #include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
// #include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
// #include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// #include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
// #include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_unitree_models/QuadrupedCom.h>
#include "ocs2_unitree_models/QuadrupedPinocchioMapping.h"
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

namespace legged
{
  using namespace ocs2;
  // using namespace legged_robot;
  using namespace switched_model;
  using namespace unitree;

  constexpr size_t GENCOORDNUMWBC = BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;

  // Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
  class WbcBase
  {
    using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
    // using kinematic_model_t = switched_model::KinematicsModelBase<scalar_t>;

  public:
    // WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);
    // WbcBase(const unitree::QuadrupedCom& ComDynamic, const kinematic_model_t &eeKinematics);
    // WbcBase(const FrameDeclaration &frameDeclaration, const unitree::QuadrupedCom &ComDynamic, const PinocchioInterface &pinocchioInterface, const PinocchioEndEffectorKinematics &eeKinematics);
    WbcBase(const FrameDeclaration &frameDeclaration, const unitree::QuadrupedCom &ComDynamic, const PinocchioInterface &pinocchioInterface);
    virtual void loadTasksSetting(const std::string &taskFile, bool verbose);

    virtual vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                            scalar_t period);
    const matrix_t &getJac() const { return j_; }
    void setacc(const vector_t& Acc){acc=Acc;};

    ros::NodeHandle nh;
    ros::Publisher debug_pub;
    ros::Publisher debug_pub2;

  protected:
    void updateMeasured(const vector_t &rbdStateMeasured);
    void updateDesired(const vector_t &stateDesired, const vector_t &inputDesired);

    size_t getNumDecisionVars() const { return numDecisionVars_; }

    Task formulateFloatingBaseEomTask();
    Task formulateTorqueLimitsTask();
    Task formulateNoContactMotionTask();
    Task formulateFrictionConeTask();
    Task formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);
    Task formulateBaseAccelTask();

    Task formulateSwingLegTask();
    Task formulateContactForceTask(const vector_t &stateDesired,const vector_t &inputDesired) const;

    size_t numDecisionVars_;
    PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
    // CentroidalModelInfo info_;

    // std::unique_ptr<kinematic_model_t> eeKinematics_;
    // std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
    // CentroidalModelPinocchioMapping mapping_;
    unitree::QuadrupedPinocchioMapping pinocchioMapping_;
    unitree::QuadrupedPinocchioMapping pinocchioMapping_Desired_;
    vector3_t eulerZyxDesired_,eulerZyxMeasured_;

    std::unique_ptr<unitree::QuadrupedCom> ComDynamic_;
    vector_t acc;

    vector_t qMeasured_, vMeasured_, inputLast_;
    matrix_t j_, dj_;
    contact_flag_t contactFlag_{};
    size_t numContacts_{};

    // Task Parameters:
    vector_t torqueLimits_;
    scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
  };

} // namespace legged
