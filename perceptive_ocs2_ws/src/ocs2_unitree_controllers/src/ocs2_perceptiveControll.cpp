/*
 * @Author: NaCl
 * @Date: 2024-04-01 13:59:14
 * @LastEditors: NaCl
 * @LastEditTime: 2024-05-17 19:18:21
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_controllers/src/ocs2_perceptiveControll.cpp
 * @Description:
 *
 */
// //
// // Created by qiayuan on 2022/6/24.
// //

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "ocs2_unitree_controllers/ocs2_perceptiveControll.h"

// #include <ocs2_centroidal_model/AccessHelperFunctions.h>
// #include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_msgs/mpc_observation.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

// #include <ocs2_mpc/LoopshapingSystemObservation.h>

// #include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
// #include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <angles/angles.h>
#include <ocs2_legged_estimation/FromTopiceEstimate.h>
#include <ocs2_legged_estimation/LinearKalmanFilter.h>
// #include <legged_wbc/HierarchicalWbc.h>
#include <ocs2_legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

//
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_sqp/SqpMpc.h>
// #include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpc.h>
#include <ocs2_quadruped_interface/QuadrupedMpc.h>
#include "std_msgs/Float32MultiArray.h"
#include <ocs2_switched_model_interface/core/Rotations.h>

namespace legged
{

    bool PerceptiveLeggedController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("Controller Initializing");
        // Initialize OCS2
        std::string descriptionName("ocs2_unitree_description");
        std::string configName("unitree");
        std::string robotName("aliengo");

        bool verbose = false;

        controller_nh.getParam("/descriptionName", descriptionName);
        controller_nh.getParam("/configName", configName);
        controller_nh.getParam("/robotName", robotName);
        controller_nh.getParam("/verbose", verbose);

        ROS_INFO_STREAM("controller_nh: " << controller_nh.getNamespace());
        ROS_INFO_STREAM("descriptionName: " << descriptionName);
        ROS_INFO_STREAM("configName: " << configName);
        ROS_INFO_STREAM("robotName: " << robotName);

        // ROS_INFO_STREAM("Task file path: " << unitree::getTaskFilePathLoopshaping(configName));

        ROS_INFO("URDF string loading");
        std::string urdfString;
        // Get parameter with the obtained descriptionName
        if (!controller_nh.getParam(descriptionName, urdfString))
        {
            ROS_ERROR_STREAM("Failed to get parameter with name: " << descriptionName);
            return 1; // Exit with error
        }
        // ROS_INFO_STREAM("urdfString: " << urdfString);

        mpcSettings = ocs2::mpc::loadSettings(unitree::getTaskFilePath(configName)); // from path of task.info to setting

        const auto sqpSettings = ocs2::sqp::loadSettings(unitree::getConfigFolder(configName) + "/multiple_shooting.info");

        leggedInterface_ = unitree::getUnitreeInterface(urdfString, unitree::getConfigFolder(configName));

        mpc_ = switched_model::getSqpMpc(*leggedInterface_, mpcSettings, sqpSettings);

        ros::NodeHandle nh;
        ROS_INFO_STREAM("nh: " << nh.getNamespace());

        const switched_model::QuadrupedInterface &quadrupedInterface = *leggedInterface_;
        auto solverModules = quadrupedInterface.getSynchronizedModules();

        // Gait
        gaitReceiver_ = std::make_shared<switched_model::GaitReceiver>(
            nh, quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), robotName);
        solverModules.push_back(gaitReceiver_);
        // Terrain Receiver
        terrainReceiver_ = std::make_shared<switched_model::TerrainReceiverSynchronizedModule>(
            quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), nh);
        solverModules.push_back(terrainReceiver_);

        // Terrain plane visualization
        terrainVisualizer_ = std::make_shared<switched_model::TerrainPlaneVisualizerSynchronizedModule>(
            quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nh, "odom");
        solverModules.push_back(terrainVisualizer_);

        // Swing planner
        swingPlanningVisualizer = std::make_shared<switched_model::SwingPlanningVisualizer>(
            quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nh, "odom");
        solverModules.push_back(swingPlanningVisualizer);

        // reference manager
        auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, quadrupedInterface.getReferenceManagerPtr());
        rosReferenceManagerPtr->subscribe(nh);
        mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

        // MPC
        mpc_->getSolverPtr()->setSynchronizedModules(solverModules);

        observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);

        // setupMrt();
        // Visualization
        visualizer_ = std::make_shared<switched_model::QuadrupedVisualizer>(
            quadrupedInterface.getKinematicModel(), quadrupedInterface.getJointNames(), quadrupedInterface.getBaseName(), nh);
        visualizer_->frameId_ = "odom";

        mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
        mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
        mpcTimer_.reset();

        controllerRunning_ = true;
        mpcThread_ = std::thread([&]()
                                 {
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
                mpcSettings.mpcDesiredFrequency_);
          } catch (const std::exception& e) {
            controllerRunning_ = false;
            ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
            stopRequest(ros::Time());
          }
        } });
        setThreadPriority(sqpSettings.threadPriority, mpcThread_);

        auto *hybridJointInterface = robot_hw->get<HybridJointInterface>();
        std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                             "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

        std::vector<std::string> contactNames3DoF{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

        for (const auto &joint_name : joint_names)
        {
            hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
        }
        auto *contactInterface = robot_hw->get<ContactSensorInterface>();
        for (const auto &name : contactNames3DoF)
        {
            contactHandles_.push_back(contactInterface->getHandle(name));
        }
        imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

        // State estimation
        setupStateEstimate(unitree::getTaskFilePath(configName), verbose, urdfString);

        currentObservation_.state = leggedInterface_->getInitialState();
        currentObservation_.input.setZero(switched_model::INPUT_DIM);

        auto frameDeclaration = frameDeclarationFromFile(unitree::getConfigFolder(configName) + "/frame_declaration.info");
        // Whole body control

        const auto &comModel = leggedInterface_->getComModel();

        // 使用 static_cast 将 comModel 转换为 QuadrupedCom 类型
        const unitree::QuadrupedCom &quadrupedCom = static_cast<const unitree::QuadrupedCom &>(comModel);

        // wbc_ = std::make_shared<WeightedWbc>(frameDeclaration, dynamic_cast<unitree::QuadrupedCom &>(const_cast<switched_model::ComModelBase<scalar_t> &>(leggedInterface_->getComModel())), createQuadrupedPinocchioInterfaceFromUrdfString(urdfString));
        wbc_ = std::make_shared<WeightedWbc>(frameDeclaration, quadrupedCom, createQuadrupedPinocchioInterfaceFromUrdfString(urdfString));
        wbc_->loadTasksSetting(unitree::getTaskFilePath(configName), verbose);

        // Safety Checker
        // safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
        controllerDebugPublisher_ = nh.advertise<std_msgs::Float32MultiArray>("/controller_debugArray", 10);
        return true;
    }

    void PerceptiveLeggedController::starting(const ros::Time &time)
    {
        ROS_INFO("Controller starting");
        // Initial state

        currentObservation_.state.setZero(STATE_DIM);
        updateStateEstimation(time, ros::Duration(0.002));
        currentObservation_.input.setZero(switched_model::INPUT_DIM);
        currentObservation_.mode = switched_model::ModeNumber::STANCE;

        // observation.input.setZero(switched_model_loopshaping::INPUT_DIM);
        TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

        // TargetTrajectories target_trajectories({currentObservation_.time}, {leggedInterface_->getInitialState()}, {currentObservation_.input});

        // Set the first observation and command and wait for optimization to finish
        mpcMrtInterface_->setCurrentObservation(currentObservation_);
        mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
        ROS_INFO_STREAM("observation.state .size " << currentObservation_.state.size());
        ROS_INFO_STREAM("observation.input .size " << currentObservation_.input.size());
        ROS_INFO_STREAM("observation \n"
                        << currentObservation_);

        ROS_INFO_STREAM("Waiting for the initial policy ...");
        while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
        {
            mpcMrtInterface_->advanceMpc();
            ros::WallRate(mpcSettings.mrtDesiredFrequency_).sleep();
        }
        ROS_INFO_STREAM("Initial policy has been received.");

        mpcRunning_ = true;
    }

    void PerceptiveLeggedController::update(const ros::Time &time, const ros::Duration &period)
    {
        // State Estimate
        updateStateEstimation(time, period);

        // Update the current state of the system
        mpcMrtInterface_->setCurrentObservation(currentObservation_);
        // Load the latest MPC policy
        mpcMrtInterface_->updatePolicy();

        // Evaluate the current policy
        vector_t optimizedState, optimizedInput;
        size_t plannedMode = 0; // The mode that is active at the time the policy is evaluated at.
        mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

        // Whole body control
        currentObservation_.input = optimizedInput;

        wbcTimer_.startTimer();
        vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
        wbcTimer_.endTimer();

        vector_t wbcF = x.segment(18, 12); // force in world with ocs2 order convert in wbc
        vector_t torqueWBC = x.tail(12);   // with ocs2 order convert in wbc

        ocs2::matrix_t jac_l = matrix_t(3 * NUM_CONTACT_POINTS, JOINT_COORDINATE_SIZE);

        for (size_t i = 0; i < NUM_CONTACT_POINTS; ++i)
        {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, JOINT_COORDINATE_SIZE);
            jac = leggedInterface_->getKinematicModel().baseToFootJacobianInBaseFrame(i, currentObservation_.state.tail<JOINT_COORDINATE_SIZE>()); // 6*12
            jac_l.block(3 * i, 0, 3, JOINT_COORDINATE_SIZE) = jac.template bottomRows<3>();
        }

        vector_t posDes = getJointPositions(optimizedState);
        vector_t velDes = getJointVelocities(optimizedInput);

        // auto jac = wbc_->getJac();

        // auto mpcJpos=ocs2::centroidal_model::getJointAngles(optimizedState,leggedInterface_->getCentroidalModelInfo());
        // auto mpcJvel=ocs2::centroidal_model::getJointVelocities(optimizedInput,leggedInterface_->getCentroidalModelInfo());
        auto forceInbase = optimizedInput.head(12); // mpc力矩
        auto torqueMPC = -jac_l.transpose() * forceInbase;
        auto rotation_matrix = switched_model::rotationMatrixBaseToOrigin(vector3_t(currentObservation_.state.head(3)));

        // auto acc = leggedInterface_->getOptimalControlProblem().dynamicsPtr->computeFlowMap(currentObservation_.time, optimizedState, optimizedInput).segment<6>(6);
        // vector6_t accwbc;
        // accwbc << acc.tail(3), acc.head(3);
        // wbc_->setacc(accwbc);
        // std::cout << "acc " << accwbc.transpose() << std::endl;
        Eigen::VectorXd forceInworld(12);
        // 循环遍历 forceInbase 中的每个三维力，并将其乘以旋转矩阵
        for (int i = 0; i < 4; ++i)
        {
            // 提取当前三维力的索引
            int start_index = i * 3;

            // 提取当前三维力向量
            Eigen::Vector3d force = forceInbase.segment<3>(start_index);

            // 将当前力向量乘以旋转矩阵
            Eigen::Vector3d rotated_force = rotation_matrix * force;

            // 将旋转后的力向量放回 forceInbase 中相应的位置
            forceInworld.segment<3>(start_index) = rotated_force;
        }

        std_msgs::Float32MultiArray array_msg;

        // // 将Eigen向量的元素逐个复制到Float32MultiArray消息的数据字段中
        for (size_t i = 0; i < torqueMPC.size(); ++i)
        {
            array_msg.data.push_back(torqueMPC(i));
        }
        for (size_t i = 0; i < forceInworld.size(); ++i)
        {
            array_msg.data.push_back(forceInworld(i));
        }
        for (size_t i = 0; i < posDes.size(); ++i)
        {
            array_msg.data.push_back(posDes(i));
        }
        for (size_t i = 0; i < velDes.size(); ++i)
        {
            array_msg.data.push_back(velDes(i));
        }
        // for (size_t i = 0; i < accwbc.size(); ++i)
        // {
        //     array_msg.data.push_back(accwbc(i));
        // }

        controllerDebugPublisher_.publish(array_msg);

        // Safety check, if failed, stop the controller
        // if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput))
        // {
        //     ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
        //     stopRequest(time);
        // }

        for (size_t j = 0; j < JOINT_COORDINATE_SIZE; ++j)
        {
            hybridJointHandles_[j].setCommand(posDes(j) * 1, velDes(j) * 1, 0, 3, torqueWBC(j) * 1);
        }

        // Visualization
        visualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

        // Publish the observation. Only needed for the command interface
        observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    }

    void PerceptiveLeggedController::updateStateEstimation(const ros::Time &time, const ros::Duration &period)
    {
        vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
        contact_flag_t contacts;
        Eigen::Quaternion<scalar_t> quat;
        contact_flag_t contactFlag;
        vector3_t angularVel, linearAccel;
        matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

        for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
        {
            jointPos(i) = hybridJointHandles_[i].getPosition();
            jointVel(i) = hybridJointHandles_[i].getVelocity();
        }
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            contactFlag[i] = contactHandles_[i].isContact();
        }
        for (size_t i = 0; i < 4; ++i)
        {
            quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
        }
        for (size_t i = 0; i < 3; ++i)
        {
            angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
            linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
        }
        for (size_t i = 0; i < 9; ++i)
        {
            orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
            angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
            linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
        }

        stateEstimate_->updateJointStates(jointPos, jointVel);
        stateEstimate_->updateContact(contactFlag);
        stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);

        measuredRbdState_ = stateEstimate_->update(time, period);

        /**
         * Switched model definition:
         *
         * state = [theta, p, w, v, q (4x)]
         * theta: EulerXYZ (3x1)
         * p: Base position in Origin frame (3x1)
         * w: Base angular velocity in Base Frame (3x1)
         * v: Base linear velocity in Base Frame (3x1)
         * q: Joint angles per leg [HAA, HFE, KFE] (3x1) [4x]
         *
         * input = [lambda (4x), qj (4x)]
         * lambda: Force at the EE [LF, RF, LH, RH] in Base Frame (3x1)
         * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
         */
        // currentObservation_.state =[theta, p, w, v, q (4x)]         // measuredRbdState_ =[euler pos jpos omega vel jvel]
        currentObservation_.time += period.toSec();
        scalar_t yawLast = currentObservation_.state(2);

        // ROS_INFO_STREAM("measuredRbdState_ "<<measuredRbdState_.transpose());
        currentObservation_.state.head(6) = measuredRbdState_.head(6); // rpy_euler pos

        currentObservation_.state.segment<6>(6) = measuredRbdState_.segment<6>(GENCOORDNUM); // baseAngularVelInBase baseLinearVelInBase

        currentObservation_.state.tail(JOINT_COORDINATE_SIZE) = measuredRbdState_.segment(6, JOINT_COORDINATE_SIZE);

        // currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
        // currentObservation_.state(2) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(2));
        currentObservation_.mode = stateEstimate_->getMode();
        // ROS_INFO_STREAM("currentObservation_ "<<currentObservation_);
    }

    PerceptiveLeggedController::~PerceptiveLeggedController()
    {
        ROS_INFO("Controller delate");
        controllerRunning_ = false;
        if (mpcThread_.joinable())
        {
            mpcThread_.join();
        }
        // if (!buffer_.empty() && false)
        // {

        //     if (mpcThread_.joinable())
        //     {
        //         mpcThread_.join();
        //     }
        //     std::cerr << "########################################################################";
        //     std::cerr << "\n### MPC Benchmarking";
        //     std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
        //     std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
        //     std::cerr << "########################################################################";
        //     std::cerr << "\n### WBC Benchmarking";
        //     std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
        //     std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
        //     std::cout << "buffer size is " << buffer_.size() << std::endl;

        //     Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

        //     if (std::ofstream logfile{logFileName_})
        //     {
        //         logfile << getLogHeader() << std::endl;
        //         for (int i = 1; i < buffer_.size(); ++i)
        //         {
        //             // std::cout << buffer_[i].transpose().format(CommaFmt) << ", \n";
        //             logfile << buffer_[i].transpose().format(CommaFmt) << ", \n";
        //         }
        //         std::cerr << "[QuadrupedLogger] Log written to '" << logFileName_ << "'\n";
        //     }
        //     else
        //     {
        //         std::cerr << "[QuadrupedLogger] Unable to open '" << logFileName_ << "'\n";
        //     }
        //     std::cerr << "\n";
        //     std::cout << "buffer size is " << buffer_.size() << std::endl;
        // }
    }

    void PerceptiveLeggedController::stopping(const ros::Time & /*time*/)
    {
        mpcRunning_ = false;
    }

    void PerceptiveLeggedController::setupStateEstimate(const std::string &taskFile, bool verbose, const std::string &urdf)
    {

        stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getKinematicModel());
        dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);
        currentObservation_.time = 0;
    }

    void PerceptiveLeggedCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/, const std::string &urdf)
    {

        stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getKinematicModel());
        currentObservation_.time = 0;
    }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::PerceptiveLeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::PerceptiveLeggedCheaterController, controller_interface::ControllerBase)
