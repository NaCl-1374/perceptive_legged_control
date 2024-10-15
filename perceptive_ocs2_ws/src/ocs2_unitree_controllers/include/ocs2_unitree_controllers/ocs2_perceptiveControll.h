/*
 * @Author: NaCl
 * @Date: 2024-04-01 13:59:14
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-16 21:11:31
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_controllers/include/ocs2_unitree_controllers/ocs2_perceptiveControll.h
 * @Description: 
 * 
 */


#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

// #include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <ocs2_legged_estimation/StateEstimateBase.h>
#include <ocs2_legged_wbc/WbcBase.h>

// #include "legged_controllers/SafetyChecker.h"
// #include <ros/node_handle.h>

// #include <ocs2_unitree_loopshaping_mpc/UnitreeLoopshapingInterface.h>
#include "ocs2_unitree_mpc/UnitreeInterface.h"


#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
//ocs2_switched_model_interface 相关
#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

#include <ocs2_quadruped_interface/SwingPlanningVisualizer.h>
#include <ocs2_quadruped_interface/TerrainPlaneVisualizer.h>
#include <ocs2_quadruped_interface/TerrainReceiver.h>

#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>


namespace legged
{
    using namespace ocs2;
    // using namespace legged_robot;
    using namespace unitree;
    // using namespace switched_model;

    class PerceptiveLeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                                   ContactSensorInterface>
    {
    public:
        PerceptiveLeggedController() = default;
        ~PerceptiveLeggedController() override;
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;
        void starting(const ros::Time &time) override;
        void stopping(const ros::Time & /*time*/) override; //{ mpcRunning_ = false; }

    protected:
        virtual void updateStateEstimation(const ros::Time &time, const ros::Duration &period);

        // virtual void setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
        //                                   bool verbose);
        // virtual void setupMpc();
        // virtual void setupMrt();
        virtual void setupStateEstimate(const std::string &taskFile, bool verbose,const std::string& urdf);

        // Interface
        // std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> leggedInterface_;
        
        std::unique_ptr<switched_model::QuadrupedInterface>leggedInterface_;
        // std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
        std::vector<HybridJointHandle> hybridJointHandles_;
        std::vector<ContactSensorHandle> contactHandles_;
        hardware_interface::ImuSensorHandle imuSensorHandle_;

        // State Estimation
        SystemObservation currentObservation_;
        vector_t measuredRbdState_;
        std::shared_ptr<StateEstimateBase> stateEstimate_;
        // std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        // std::shared_ptr<SafetyChecker> safetyChecker_;

        // Nonlinear MPC
        std::shared_ptr<MPC_BASE> mpc_;
        std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

        // sync SloverModules
        std::shared_ptr<switched_model::GaitReceiver> gaitReceiver_;
        std::shared_ptr<switched_model::TerrainReceiverSynchronizedModule> terrainReceiver_;
        std::shared_ptr<switched_model::TerrainPlaneVisualizerSynchronizedModule> terrainVisualizer_;
        std::shared_ptr<switched_model::SwingPlanningVisualizer> swingPlanningVisualizer;


        // std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
        ros::Publisher observationPublisher_;
        ros::Publisher controllerDebugPublisher_;

        std::shared_ptr<switched_model::QuadrupedVisualizer> visualizer_;

        // logger
        // void addLine(const ocs2::SystemObservation &observation, const vector_t &additionalColumns = vector_t());

        // static std::vector<std::string> namesPerLeg(const std::string &prefix, const std::vector<std::string> &postfixes);
        // std::string getLogHeader() const;
        // int getNumColumns() const;
        // std::string logFileName_;
        // std::vector<std::string> additionalColumns_;

        

    private:
        std::thread mpcThread_;
        std::atomic_bool controllerRunning_{}, mpcRunning_{};
        benchmark::RepeatedTimer mpcTimer_;
        benchmark::RepeatedTimer wbcTimer_;
        std::vector<vector_t> buffer_;

        ocs2::mpc::Settings mpcSettings;
    };

    class PerceptiveLeggedCheaterController : public PerceptiveLeggedController
    {
    protected:
        void setupStateEstimate(const std::string &taskFile, bool verbose,const std::string& urdf) override;
    };

} // namespace legged
