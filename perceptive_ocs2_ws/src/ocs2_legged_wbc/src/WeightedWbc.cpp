/*
 * @Author: NaCl
 * @Date: 2024-04-08 14:20:31
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-09 16:21:55
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_wbc/src/WeightedWbc.cpp
 * @Description:
 *
 */
//
// Created by qiayuan on 22-12-23.
//

#include "ocs2_legged_wbc/WeightedWbc.h"

#include <qpOASES.hpp>

namespace legged
{

  vector_t WeightedWbc::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                               scalar_t period)
  {
    // std::cout << "update des and mesure" << std::endl;

    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

    // Constraints
    // std::cout << "formulateConstraints" << std::endl;

    Task constraints = formulateConstraints();
    size_t numConstraints = constraints.b_.size() + constraints.f_.size();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
    vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_; // clang-format on

    //   // Cost
    Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

    // Solve
    auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);
    int nWsr = 20;

    qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    vector_t qpSol(getNumDecisionVars());

    qpProblem.getPrimalSolution(qpSol.data());

    // 转成ocs2顺序的 qpSol=[ddq fx4 qx4]
    // qpSol.segment<JOINT_COORDINATE_SIZE>(BASE_COORDINATE_SIZE) =
    //     pinocchioMapping_.getOcs2JointVector(switched_model::joint_coordinate_t(qpSol.segment<12>(6)));

    qpSol.segment<3 * NUM_CONTACT_POINTS>(GENCOORDNUMWBC) =
        pinocchioMapping_.getOcs2JointVector(switched_model::joint_coordinate_t(qpSol.segment<3 * NUM_CONTACT_POINTS>(GENCOORDNUMWBC)));

    qpSol.segment<JOINT_COORDINATE_SIZE>(GENCOORDNUMWBC + 3 * NUM_CONTACT_POINTS) =
        pinocchioMapping_.getOcs2JointVector(switched_model::joint_coordinate_t(qpSol.segment<JOINT_COORDINATE_SIZE>(GENCOORDNUMWBC + 3 * NUM_CONTACT_POINTS)));

    // 创建一个Float32MultiArray消息对象
    std_msgs::Float32MultiArray array_msg;

    // 将Eigen向量的大小设置为Float32MultiArray消息的数据字段大小
    array_msg.data.resize(qpSol.size());

    // 将Eigen向量的元素逐个复制到Float32MultiArray消息的数据字段中
    for (size_t i = 0; i < qpSol.size(); ++i)
    {
      array_msg.data[i] = qpSol(i);
    }
    // 发布消息
    debug_pub.publish(array_msg);

    return qpSol;
  }

  Task WeightedWbc::formulateConstraints()
  {
    // return formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
    // return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateNoContactMotionTask();

    return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  }

  Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
  {
    // return formulateSwingLegTask() * weightSwingLeg_  +
    //        formulateContactForceTask(inputDesired) * weightContactForce_;
    // return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
    //        formulateContactForceTask(inputDesired) * weightContactForce_;

    return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
           formulateContactForceTask(stateDesired,inputDesired) * weightContactForce_;
  }

  void WeightedWbc::loadTasksSetting(const std::string &taskFile, bool verbose)
  {
    WbcBase::loadTasksSetting(taskFile, verbose);

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "weight.";
    if (verbose)
    {
      std::cerr << "\n #### WBC weight:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
    loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
    loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
  }

} // namespace legged
