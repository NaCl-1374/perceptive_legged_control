/*
 * @Author: NaCl
 * @Date: 2024-04-01 12:31:17
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-06 16:58:26
 * @FilePath: /perceptive_ocs2_ws/src/legged_control/legged_controllers/src/LeggedController.cpp
 * @Description: 
 * 
 */
//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

//logger
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <pinocchio/algorithm/frames.hpp>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <fstream>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  //logger
  ROS_INFO("%s", getLogHeader().c_str());

  logFileName_="/tmp/legged_control/QuadrupedGazeboNodeLog.txt";
  std::vector<std::string> MpcJpos=namesPerLeg("MpcjointAngle", {"HAA", "HFE", "KFE"});
  std::vector<std::string> MpcJvel=namesPerLeg("MpcjointVel", {"HAA", "HFE", "KFE"});
  std::vector<std::string> MpcJtor=namesPerLeg("MpcjointTor", {"HAA", "HFE", "KFE"});

  // std::vector<std::string> wbcJpos=namesPerLeg("WbcjointAngle", {"HAA", "HFE", "KFE"});
  // std::vector<std::string> wbcJvel=namesPerLeg("WbcjointVel", {"HAA", "HFE", "KFE"});
  std::vector<std::string> WbcJtor=namesPerLeg("WbcjointTor", {"HAA", "HFE", "KFE"});
  std::vector<std::string> WbcForce=namesPerLeg("WbcForce", {"HAA", "HFE", "KFE"});

  additionalColumns_.insert(additionalColumns_.end(), MpcJpos.begin(), MpcJpos.end());
  additionalColumns_.insert(additionalColumns_.end(), MpcJvel.begin(), MpcJvel.end());
  additionalColumns_.insert(additionalColumns_.end(), MpcJtor.begin(), MpcJtor.end());

  // additionalColumns_.insert(additionalColumns_.end(), wbcJpos.begin(), wbcJpos.end());
  // additionalColumns_.insert(additionalColumns_.end(), wbcJvel.begin(), wbcJvel.end());
  additionalColumns_.insert(additionalColumns_.end(), WbcJtor.begin(), WbcJtor.end());
  additionalColumns_.insert(additionalColumns_.end(), WbcForce.begin(), WbcForce.end());

  // ROS_INFO("%s", getLogHeader().c_str());

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  wbcTimer_.endTimer();

  vector_t wbcF=x.segment(18,12);
  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());


  auto jac=wbc_->getJac();

  // auto mpcJpos=ocs2::centroidal_model::getJointAngles(optimizedState,leggedInterface_->getCentroidalModelInfo());
  // auto mpcJvel=ocs2::centroidal_model::getJointVelocities(optimizedInput,leggedInterface_->getCentroidalModelInfo());
  auto forceInbase=optimizedInput.head(12);// mpc力矩
  auto Tor=-jac.transpose()*forceInbase;

  vector_t TorMpc=Tor.tail(12);
  vector_t combined=vector_t::Zero(additionalColumns_.size());
    combined<<posDes,velDes,TorMpc,torque,wbcF;

  // combined<<mpcJpos,mpcJvel,TorMpc,posDes,velDes,torque;
  
  addLine(currentObservation_,combined);


  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
  }

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;



  if (!buffer_.empty()){

    if (mpcThread_.joinable()) {
      mpcThread_.join();
    }
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
    std::cerr << "\n### WBC Benchmarking";
    std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cout<<"buffer size is "<<buffer_.size()<<std::endl;

    Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

    if (std::ofstream logfile{logFileName_}) {
      logfile << getLogHeader() << std::endl;
      for (int i = 1; i < buffer_.size(); ++i) {
        // std::cout << buffer_[i].transpose().format(CommaFmt) << ", \n";
        logfile << buffer_[i].transpose().format(CommaFmt) << ", \n";
      }
      std::cerr << "[QuadrupedLogger] Log written to '" << logFileName_ << "'\n";
    } else {
      std::cerr << "[QuadrupedLogger] Unable to open '" << logFileName_ << "'\n";
    }
    std::cerr << "\n";
    std::cout<<"buffer size is "<<buffer_.size()<<std::endl;
  }
}

void LeggedController::stopping(const ros::Time& /*time*/) {
  mpcRunning_ = false; 

   
  }

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

std::string LeggedController::getLogHeader() const {
  std::string delim = ", ";
  std::stringstream header;
  // clang-format off
  header <<
         "time" << delim <<
         "contactflag_LF" << delim <<
         "contactflag_RF" << delim <<
         "contactflag_LH" << delim <<
         "contactflag_RH" << delim <<
         "base_positionInWorld_x" << delim <<
         "base_positionInWorld_y" << delim <<
         "base_positionInWorld_z" << delim <<
         "base_quaternion_w" << delim <<
         "base_quaternion_x" << delim <<
         "base_quaternion_y" << delim <<
         "base_quaternion_z" << delim <<
         "base_linearvelocityInBase_x" << delim <<
         "base_linearvelocityInBase_y" << delim <<
         "base_linearvelocityInBase_z" << delim <<
         "base_angularvelocityInBase_x" << delim <<
         "base_angularvelocityInBase_y" << delim <<
         "base_angularvelocityInBase_z" << delim;
  // clang-format on
  for (const auto& name : namesPerLeg("jointAngle", {"HAA", "HFE", "KFE"})) {
    header << name << delim;
  }
  for (const auto& name : namesPerLeg("jointVelocity", {"HAA", "HFE", "KFE"})) {
    header << name << delim;
  }
  for (const auto& name : namesPerLeg("contactForcesInWorld", {"x", "y", "z"})) {
    header << name << delim;
  }
  for (const auto& name : additionalColumns_) {
    header << name << delim;
  }
  return header.str();
}

int LeggedController::getNumColumns() const {
  auto CentroidalModelInfo= leggedInterface_->getCentroidalModelInfo();
  int numColumns = 0;
  numColumns += 1;                          // time
  numColumns += CentroidalModelInfo.numThreeDofContacts;         // contactFlags
  numColumns += 6 + 1;   // base pose (with quaternion)
  numColumns += 6;       // base twist
  numColumns += CentroidalModelInfo.actuatedDofNum;      // joint angles
  numColumns += CentroidalModelInfo.actuatedDofNum;      // joint velocities
  numColumns += 3 * CentroidalModelInfo.numThreeDofContacts;     // contact forces
  numColumns += additionalColumns_.size();  // extra columns
  return numColumns;
}
feet_array_t<vector3_t> toArray(const Eigen::Matrix<scalar_t, 12, 1> & valuesAsVector) {
  return {valuesAsVector.template segment<3>(0), valuesAsVector.template segment<3>(3), valuesAsVector.template segment<3>(6),
          valuesAsVector.template segment<3>(9)};
}

void LeggedController::addLine(const ocs2::SystemObservation& observation, const vector_t& additionalColumns) {
  if (!buffer_.empty() && observation.time == buffer_.back()[0]) {  // time is same as last one
    return;
  }
  vector_t state(observation.state);
  vector_t input(observation.input);
  auto CentroidalModelInfo= leggedInterface_->getCentroidalModelInfo();

  // Extract elements from state
  const vector_t basePose = ocs2::centroidal_model::getBasePose(state,CentroidalModelInfo);
  const auto basePosition = basePose.head<3>();
  const vector_t baseLocalVelocities = ocs2::centroidal_model::getBasePose(state,CentroidalModelInfo);
  const auto baseAngularVelocities = baseLocalVelocities.tail<3>();
  const auto baseLinearVelocities = baseLocalVelocities.head<3>();
  const vector_t qJoints = ocs2::centroidal_model::getJointAngles(state,CentroidalModelInfo);
  const vector_t dqJoints = ocs2::centroidal_model::getJointVelocities(input,CentroidalModelInfo);
  const Eigen::Matrix3d o_R_b = ocs2::getRotationMatrixFromZyxEulerAngles(vector3_t(basePose.tail<3>()));
  const auto quat = ocs2::matrixToQuaternion(o_R_b);

  // Contact state
  const contact_flag_t contactFlags = modeNumber2StanceLeg(observation.mode);

  // Forces
  Eigen::Matrix<scalar_t, 12, 1> forceInputs = input.head<3 * 4>();
  const auto contactForcesInBase = toArray(forceInputs);
  feet_array_t<vector3_t> contactForcesInWorld;
  for (size_t i = 0; i < CentroidalModelInfo.numThreeDofContacts; i++) {
    contactForcesInWorld[i] = o_R_b * contactForcesInBase[i];
  }

  // Fill log
  vector_t logEntry(getNumColumns());
  // clang-format off
  logEntry <<
      observation.time,
      static_cast<double>(contactFlags[0]),
      static_cast<double>(contactFlags[1]),
      static_cast<double>(contactFlags[2]),
      static_cast<double>(contactFlags[3]),
      basePosition,
      quat.w(),
      quat.x(),
      quat.y(),
      quat.z(),
      baseLinearVelocities,
      baseAngularVelocities,
      qJoints,
      dqJoints,
      contactForcesInBase[0],
      contactForcesInBase[1],
      contactForcesInBase[2],
      contactForcesInBase[3],
      additionalColumns;
  // clang-format on
  buffer_.push_back(std::move(logEntry));

  // std::cout<<buffer_.size()<<std::endl;

}

std::vector<std::string> LeggedController::namesPerLeg(const std::string& prefix, const std::vector<std::string>& postfixes) {
  std::vector<std::string> names;
  for (const auto& legName : std::vector<std::string>{"LF", "LH", "RF", "RH"}) {
    for (const auto& postfix : postfixes) {
      names.emplace_back(prefix + "_" + legName + "_" + postfix);
    }
  }
  return names;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
