//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "ocs2_legged_wbc/WbcBase.h"

// #include <ocs2_centroidal_model/AccessHelperFunctions.h>
// #include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged
{
  // WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
  // WbcBase::WbcBase(const FrameDeclaration &frameDeclaration, const unitree::QuadrupedCom &ComDynamic, const PinocchioInterface &pinocchioInterface, const PinocchioEndEffectorKinematics &eeKinematics)
  //     : pinocchioInterfaceMeasured_(pinocchioInterface),
  //       pinocchioInterfaceDesired_(pinocchioInterface),
  //       ComDynamic_(ComDynamic.clone()),
  //       // mapping_(info_),
  //       pinocchioMapping_(frameDeclaration, pinocchioInterfaceMeasured_),
  //       pinocchioMapping_Desired_(frameDeclaration, pinocchioInterfaceDesired_),
  //       inputLast_(vector_t::Zero(INPUT_DIM)),
  //       eeKinematics_(eeKinematics.clone())
  WbcBase::WbcBase(const FrameDeclaration &frameDeclaration, const unitree::QuadrupedCom &ComDynamic, const PinocchioInterface &pinocchioInterface)
      : pinocchioInterfaceMeasured_(pinocchioInterface),
        pinocchioInterfaceDesired_(pinocchioInterface),
        ComDynamic_(ComDynamic.clone()),
        // mapping_(info_),
        pinocchioMapping_(frameDeclaration, pinocchioInterfaceMeasured_),
        pinocchioMapping_Desired_(frameDeclaration, pinocchioInterfaceDesired_),
        inputLast_(vector_t::Zero(INPUT_DIM))
  {
    numDecisionVars_ = GENCOORDNUMWBC + 3 * NUM_CONTACT_POINTS + JOINT_COORDINATE_SIZE;
    qMeasured_ = vector_t(GENCOORDNUMWBC + 1);
    vMeasured_ = vector_t(GENCOORDNUMWBC);
    debug_pub = nh.advertise<std_msgs::Float32MultiArray>("/wbc_debugArray", 10);
    debug_pub2 = nh.advertise<std_msgs::Float32MultiArray>("/wbc_debugArray2", 10);
    std::cout << "PinocchioJointNames"
              << "\n";
    for (const auto &str : pinocchioMapping_.getPinocchioJointNames())
      std::cout << str << "\t";

    std::cout << "\n Ocs2JointNames"
              << "\n";
    for (const auto &str : getJointNames(frameDeclaration))
      std::cout << str << "\t";
    std::cout << "\n ";
  }

  vector_t WbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                           scalar_t /*period*/)
  {
    contactFlag_ = switched_model::modeNumber2StanceLeg(mode); // {LF, RF, LH, RH}
    numContacts_ = 0;
    for (bool flag : contactFlag_)
    {
      if (flag)
      {
        numContacts_++;
      }
    }

    updateMeasured(rbdStateMeasured);
    updateDesired(stateDesired, inputDesired);

    return {};
  }

  void WbcBase::updateMeasured(const vector_t &rbdStateMeasured) // measuredRbdState_ =[euler pos jpos omega vel jvel]
  {

    // std::cout << "updateMeasured " << std::endl;
    vector_t ocs2State, ocs2Input; // state =[theta, p, w, v, q (4x)]
    ocs2State.setZero(STATE_DIM);
    ocs2State << rbdStateMeasured.head<BASE_COORDINATE_SIZE>(),
        rbdStateMeasured.segment<BASE_COORDINATE_SIZE>(GENCOORDNUMWBC),
        rbdStateMeasured.segment<JOINT_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
    // std::cout << "updateMeasured 1" << std::endl;

    ocs2Input.setZero(INPUT_DIM);
    ocs2Input.segment<JOINT_COORDINATE_SIZE>(NUM_CONTACT_POINTS * 3) = rbdStateMeasured.tail(JOINT_COORDINATE_SIZE);

    switched_model::joint_coordinate_t ocsJpos = getJointPositions(ocs2State);
    switched_model::joint_coordinate_t ocsJvel = getJointVelocities(ocs2Input);

    auto basepose = switched_model::getBasePose(ocs2State);
    auto BaseLocalVel = switched_model::getBaseLocalVelocities(ocs2State);

    vector3_t eulerZYX = rotationMatrixBaseToOrigin(getOrientation(basepose)).eulerAngles(2, 1, 0);
    ocs2::makeEulerAnglesUnique<scalar_t>(eulerZYX);
    eulerZyxMeasured_ = eulerZYX;
    // std::cout << "updateMeasured 2" << std::endl;

    qMeasured_ = ComDynamic_->getPinnochioConfiguration(basepose, ocsJpos);
    vMeasured_ = ComDynamic_->getPinnochioVelocity(BaseLocalVel, ocsJvel);
    // std::cout << "updateMeasured 3" << std::endl;

    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();
    // std::cout << "updateMeasured 4" << std::endl;

    // For floating base EoM task
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    // std::cout << "updateMeasured 5" << std::endl;

    pinocchio::computeJointJacobians(model, data);
    // std::cout << "updateMeasured 6" << std::endl;

    pinocchio::updateFramePlacements(model, data);
    // std::cout << "updateMeasured 7" << std::endl;

    pinocchio::crba(model, data, qMeasured_);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

    j_ = matrix_t(3 * NUM_CONTACT_POINTS, GENCOORDNUMWBC);

    // std::cout << "getFrameJacobian \n"
    //           << std::endl;

    /**
     * Ocs2FootIndex 0 -> 0 pinocchio idx
     * Ocs2FootIndex 1 -> 2 pinocchio idx
     * Ocs2FootIndex 2 -> 1 pinocchio idx
     * Ocs2FootIndex 3 -> 3 pinocchio idx
     * pinocchio idx 0 -> 0 Ocs2FootIndex
     * pinocchio idx 1 -> 2 Ocs2FootIndex
     * pinocchio idx 2 -> 1 Ocs2FootIndex
     * pinocchio idx 3 -> 3 Ocs2FootIndex
     */
    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) // pinocchio的足端顺序排列雅可比 ocs2的是LF RF LH RH pinocchoi 的是  LF LH RF RH
    {
      // std::cout << " Ocs2FootIndex " << i << " -> " << pinocchioMapping_.getPinocchioFootIndex(i) << " pinocchio idx " << std::endl;

      // std::cout << "pinocchio idx " << i << " -> " << pinocchioMapping_.getOcs2FootIndex(i) << " Ocs2FootIndex " << std::endl;
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, GENCOORDNUMWBC);
      pinocchio::getFrameJacobian(model, data, pinocchioMapping_.getFootFrameId(pinocchioMapping_.getOcs2FootIndex(i)), pinocchio::LOCAL_WORLD_ALIGNED, jac);
      j_.block(3 * i, 0, 3, GENCOORDNUMWBC) = jac.template topRows<3>();
    }

    // For not contact motion task
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    dj_ = matrix_t(3 * NUM_CONTACT_POINTS, GENCOORDNUMWBC);
    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i)
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, GENCOORDNUMWBC);
      pinocchio::getFrameJacobianTimeVariation(model, data, pinocchioMapping_.getFootFrameId(pinocchioMapping_.getOcs2FootIndex(i)), pinocchio::LOCAL_WORLD_ALIGNED, jac);
      dj_.block(3 * i, 0, 3, GENCOORDNUMWBC) = jac.template topRows<3>();
    }
    // std::cout << "updateMeasured over" << std::endl;
  }

  // * state = [theta(xyz), p, w, v, q (4x)]
  void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired)
  {

    // std::cout << "updateDesired " << std::endl;

    const auto &model = pinocchioInterfaceDesired_.getModel();
    auto &data = pinocchioInterfaceDesired_.getData();

    switched_model::joint_coordinate_t ocsJpos = getJointPositions(stateDesired);
    switched_model::joint_coordinate_t ocsJvel = getJointVelocities(inputDesired);

    auto basepose = switched_model::getBasePose(stateDesired);
    auto BaseLocalVel = switched_model::getBaseLocalVelocities(stateDesired);

    vector3_t eulerZYX = rotationMatrixBaseToOrigin(getOrientation(basepose)).eulerAngles(2, 1, 0);
    ocs2::makeEulerAnglesUnique<scalar_t>(eulerZYX);

    eulerZyxDesired_ = eulerZYX;
    vector_t qDesired, vDesired;
    qDesired = ComDynamic_->getPinnochioConfiguration(basepose, ocsJpos);
    vDesired = ComDynamic_->getPinnochioVelocity(BaseLocalVel, ocsJvel);

    pinocchio::forwardKinematics(model, data, qDesired);
    pinocchio::computeJointJacobians(model, data, qDesired);
    // pinocchio::updateFramePlacements(model, data);

    // updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);

    // const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
    pinocchio::forwardKinematics(model, data, qDesired, vDesired);

    // std::cout << "updateDesired over " << std::endl;
  }

  Task WbcBase::formulateFloatingBaseEomTask()
  {
    // std::cout << "formulateFloatingBaseEomTask" << std::endl;
    auto &data = pinocchioInterfaceMeasured_.getData();

    matrix_t s(JOINT_COORDINATE_SIZE, GENCOORDNUMWBC);
    s.block(0, 0, JOINT_COORDINATE_SIZE, 6).setZero();
    s.block(0, 6, JOINT_COORDINATE_SIZE, JOINT_COORDINATE_SIZE).setIdentity();

    matrix_t a = (matrix_t(GENCOORDNUMWBC, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
    vector_t b = -data.nle;

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateTorqueLimitsTask()
  {
    // std::cout << "formulateTorqueLimitsTask" << std::endl;

    matrix_t d(2 * JOINT_COORDINATE_SIZE, numDecisionVars_);
    d.setZero();
    matrix_t i = matrix_t::Identity(JOINT_COORDINATE_SIZE, JOINT_COORDINATE_SIZE);
    d.block(0, GENCOORDNUMWBC + 3 * NUM_CONTACT_POINTS, JOINT_COORDINATE_SIZE, JOINT_COORDINATE_SIZE) = i;
    d.block(JOINT_COORDINATE_SIZE, GENCOORDNUMWBC + 3 * NUM_CONTACT_POINTS, JOINT_COORDINATE_SIZE,
            JOINT_COORDINATE_SIZE) = -i;
    vector_t f(2 * JOINT_COORDINATE_SIZE);
    for (size_t l = 0; l < 2 * JOINT_COORDINATE_SIZE / 3; ++l)
    {
      f.segment<3>(3 * l) = torqueLimits_;
    }

    return {matrix_t(), vector_t(), d, f};
  }

  Task WbcBase::formulateNoContactMotionTask()
  {
    // std::cout << "formulateNoContactMotionTask" << std::endl;

    matrix_t a(3 * numContacts_, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) // pinocchio的顺序，
    {
      if (contactFlag_[pinocchioMapping_.getOcs2FootIndex(i)]) // contactFlag_ {LF, RF, LH, RH} 是ocs2的顺序，需要pinocchoi的 LF LH RF RH 转 ocs2
      {
        a.block(3 * j, 0, 3, GENCOORDNUMWBC) = j_.block(3 * i, 0, 3, GENCOORDNUMWBC);
        b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, GENCOORDNUMWBC) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateFrictionConeTask()
  {
    // std::cout << "formulateFrictionConeTask" << std::endl;

    matrix_t a(3 * (NUM_CONTACT_POINTS - numContacts_), numDecisionVars_);
    a.setZero();
    size_t j = 0;
    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) // pinocchio的顺序，
    {
      if (!contactFlag_[pinocchioMapping_.getOcs2FootIndex(i)]) // contactFlag_ {LF, RF, LH, RH} 是ocs2的顺序，需要pinocchoi的 LF LH RF RH 转 ocs2
      {
        a.block(3 * j++, GENCOORDNUMWBC + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
      }
    }
    vector_t b(a.rows());
    b.setZero();
    matrix_t frictionPyramic(5, 3); // clang-format off
  frictionPyramic << 0, 0, -1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_; // clang-format on

    matrix_t d(5 * numContacts_ + 3 * (NUM_CONTACT_POINTS - numContacts_), numDecisionVars_);
    d.setZero();
    j = 0;
    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i)
    {
      if (contactFlag_[pinocchioMapping_.getOcs2FootIndex(i)])
      {
        d.block(5 * j++, GENCOORDNUMWBC + 3 * i, 5, 3) = frictionPyramic;
      }
    }
    vector_t f = Eigen::VectorXd::Zero(d.rows());

    return {a, b, d, f};
  }

  Task WbcBase::formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
  {
    // std::cout << "formulateBaseAccelTask" << std::endl;

    std_msgs::Float32MultiArray array_msg;

    auto basepose = switched_model::getBasePose(stateDesired);
    auto BaseLocalVel = switched_model::getBaseLocalVelocities(stateDesired);

    auto JointPos = switched_model::getJointPositions(stateDesired);
    auto JointVel = switched_model::getJointVelocities(inputDesired);

    std::vector<vector3_t> posDesired;
    std::vector<vector3_t> velDesired;

    pinocchio::Data &dataDesired = pinocchioInterfaceDesired_.getData();
    const pinocchio::Model &modelDesired = pinocchioInterfaceDesired_.getModel();
    pinocchio::updateFramePlacements(modelDesired, dataDesired);

    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) // pinocchio的足端顺序
    {
      size_t frameId = pinocchioMapping_.getFootFrameId(pinocchioMapping_.getOcs2FootIndex(i)); // 得到pinocchio足端id
      posDesired.emplace_back(dataDesired.oMf[frameId].translation() - basepose.tail(3));
    }

    auto forceinbase = pinocchioMapping_Desired_.getPinocchioJointVector(joint_coordinate_t(inputDesired.head(JOINT_COORDINATE_SIZE)));

    // contact JacobianTransposeLambda 这里用的都是xyz欧拉角
    base_coordinate_s_t<scalar_t> JcTransposeLambda = base_coordinate_s_t<scalar_t>::Zero();
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++)
    {
      vector3_s_t<scalar_t> baseToFootInBase = rotateVectorOriginToBase(posDesired[i], getOrientation(basepose));
      vector3_s_t<scalar_t> contactForce = forceinbase.segment<3>(3 * i);

      JcTransposeLambda.head(3) += baseToFootInBase.cross(contactForce);
      JcTransposeLambda.tail(3) += contactForce;
    }
    Vector6 ocs2BaseLocalAccelerations = ComDynamic_->calculateBaseLocalAccelerations(basepose, BaseLocalVel, JointPos, JointVel, joint_coordinate_s_t<scalar_t>::Zero(), JcTransposeLambda);

    // 将Eigen向量的元素逐个复制到Float32MultiArray消息的数据字段中
    for (size_t i = 0; i < ocs2BaseLocalAccelerations.size(); ++i)
    {
      array_msg.data.push_back(ocs2BaseLocalAccelerations(i));
    }

    for (size_t i = 0; i < ocs2BaseLocalAccelerations.size(); ++i)
    {
      array_msg.data.push_back(ocs2BaseLocalAccelerations(i));
    }

    debug_pub2.publish(array_msg);
    matrix_t a(6, numDecisionVars_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

    inputLast_ = inputDesired;

    Vector6 b;
    b << ocs2BaseLocalAccelerations.tail(3), ocs2BaseLocalAccelerations.head(3);
    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateBaseAccelTask()
  {
    matrix_t a(6, numDecisionVars_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);
    Vector6 b;
    b << acc;
    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateSwingLegTask() // 从ocs2那里更新的期望足端位置
  {

    // std::cout << "formulateSwingLegTask" << std::endl;

    std::vector<vector3_t> posMeasured;
    std::vector<vector3_t> velMeasured;
    std::vector<vector3_t> posDesired;
    std::vector<vector3_t> velDesired;

    pinocchio::Data &dataMeasured = pinocchioInterfaceMeasured_.getData();
    const pinocchio::Model &modelMeasured = pinocchioInterfaceMeasured_.getModel();

    pinocchio::updateFramePlacements(modelMeasured, dataMeasured);
    // std::cout << "updateFramePlacements modelMeasured" << std::endl;

    pinocchio::Data &dataDesired = pinocchioInterfaceDesired_.getData();
    const pinocchio::Model &modelDesired = pinocchioInterfaceDesired_.getModel();

    pinocchio::updateFramePlacements(modelDesired, dataDesired);
    // std::cout << "updateFramePlacements modelDesired" << std::endl;

    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i) // pinocchio的足端顺序
    {
      size_t frameId = pinocchioMapping_.getFootFrameId(pinocchioMapping_.getOcs2FootIndex(i)); // 得到pinocchio足端id
      posMeasured.emplace_back(dataMeasured.oMf[frameId].translation());
      velMeasured.emplace_back(pinocchio::getFrameVelocity(modelMeasured, dataMeasured, frameId, rf).linear());

      posDesired.emplace_back(dataDesired.oMf[frameId].translation());
      velDesired.emplace_back(pinocchio::getFrameVelocity(modelDesired, dataDesired, frameId, rf).linear());
    }
    // eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    // std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
    // std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

    // eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    // std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
    // std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

    matrix_t a(3 * (NUM_CONTACT_POINTS - numContacts_), numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i)
    {
      if (!contactFlag_[pinocchioMapping_.getOcs2FootIndex(i)])
      {
        vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
        a.block(3 * j, 0, 3, GENCOORDNUMWBC) = j_.block(3 * i, 0, 3, GENCOORDNUMWBC);
        b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, GENCOORDNUMWBC) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateContactForceTask(const vector_t &stateDesired, const vector_t &inputDesired) const
  {
    // std::cout << "formulateContactForceTask" << std::endl;
    auto basepose = switched_model::getBasePose(stateDesired);
    auto wRb = rotationMatrixBaseToOrigin(getOrientation(basepose)); // xyz欧拉角
    matrix_t a(3 * NUM_CONTACT_POINTS, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();

    for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i)
    {
      a.block(3 * i, GENCOORDNUMWBC + 3 * i, 3, 3) = matrix_t::Identity(3, 3) * wRb.transpose();
    }
    // b = inputDesired.head(a.rows());
    b = pinocchioMapping_Desired_.getPinocchioJointVector(switched_model::joint_coordinate_t(inputDesired.head(a.rows())));

    return {a, b, matrix_t(), vector_t()};
  }

  void WbcBase::loadTasksSetting(const std::string &taskFile, bool verbose)
  {

    // Load task file
    torqueLimits_ = vector_t(JOINT_COORDINATE_SIZE / 4);
    loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
    if (verbose)
    {
      std::cerr << "\n #### Torque Limits Task:";
      std::cerr << "\n #### =============================================================================\n";
      std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
      std::cerr << " #### =============================================================================\n";
    }
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "frictionConeTask.";
    if (verbose)
    {
      std::cerr << "\n #### Friction Cone Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
    if (verbose)
    {
      std::cerr << " #### =============================================================================\n";
    }
    prefix = "swingLegTask.";
    if (verbose)
    {
      std::cerr << "\n #### Swing Leg Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
  }

} // namespace legged
