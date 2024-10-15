/*
 * @Author: NaCl
 * @Date: 2024-03-09 18:23:47
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-12 16:34:40
 * @FilePath: /OCS2_ws/src/ocs2/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_quadruped_interface/include/ocs2_quadruped_interface/TerrainPlaneVisualizer.h
 * @Description: 
 * 
 */
//
// Created by rgrandia on 30.04.20.
//

#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

namespace switched_model {

class TerrainPlaneVisualizer {
 public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "world";  // Frame name all messages are published in
  double planeWidth_ = 1.5;
  double planeLength_ = 1.5;
  double planeThickness_ = 0.005;
  double planeAlpha_ = 0.5;

  explicit TerrainPlaneVisualizer(ros::NodeHandle& nodeHandle);

  void update(scalar_t time, const TerrainPlane& terrainPlane);

 private:
  ros::Publisher terrainPublisher_;
};

class TerrainPlaneVisualizerSynchronizedModule : public ocs2::SolverSynchronizedModule {
 public:
  TerrainPlaneVisualizerSynchronizedModule(const SwingTrajectoryPlanner& swingTrajectoryPlanner, ros::NodeHandle& nodeHandle,const std::string frameId_ = "world");

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::ReferenceManagerInterface& referenceManager) override{};

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override;

 private:
  const SwingTrajectoryPlanner* swingTrajectoryPlanner_;
  TerrainPlaneVisualizer planeVisualizer_;
};

}  // namespace switched_model
