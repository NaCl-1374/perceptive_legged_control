/*
 * @Author: NaCl
 * @Date: 2024-04-19 10:40:14
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-19 11:11:16
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_mpc/include/ocs2_unitree_mpc/visualization/LeggedSelfCollisionVisualization.h
 * @Description:
 *
 */
//
// Created by qiayuan on 23-1-30.
//

#pragma once
#include <ros/ros.h>

#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <utility>
#include "ocs2_unitree_models/QuadrupedPinocchioMapping.h"
#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
namespace unitree
{

  using namespace ocs2;

  class LeggedSelfCollisionVisualization : public GeometryInterfaceVisualization
  {
  public:
    LeggedSelfCollisionVisualization(const FrameDeclaration &frameDeclaration, PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                     ros::NodeHandle &nh, scalar_t maxUpdateFrequency = 50.0)
        : pinocchioMapping_(frameDeclaration, pinocchioInterface),

          // mappingPtr_(mapping.clone()),
          GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), nh, "odom"),
          lastTime_(std::numeric_limits<scalar_t>::lowest()),
          minPublishTimeDifference_(1.0 / maxUpdateFrequency)
    {
    }
    void update(const vector_t &observation, const scalar_t time)
    {
      if (time - lastTime_ > minPublishTimeDifference_)
      {
        lastTime_ = time;

        publishDistances(pinocchioMapping_.getPinocchioJointVector(switched_model::joint_coordinate_t(observation.tail(12))));
      }
    }

  private:
    // std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
    unitree::QuadrupedPinocchioMapping pinocchioMapping_;

    scalar_t lastTime_;
    scalar_t minPublishTimeDifference_;
  };

  class SelfCollisionVisualizationSynchronizedModule : public ocs2::SolverSynchronizedModule
  {
  public:
    SelfCollisionVisualizationSynchronizedModule(const FrameDeclaration &frameDeclaration, PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                                 ros::NodeHandle &nodeHandle, const std::string frameId_ = "world")
        : pinocchioMapping_(frameDeclaration, pinocchioInterface),
          SelfCollisionVisualizer_(std::move(pinocchioInterface), std::move(geometryInterface), nodeHandle, frameId_),
          minPublishTimeDifference_(1.0 / 50) {}

    void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                      const ocs2::ReferenceManagerInterface &referenceManager) override
    {
      if (initTime - lastTime_ > minPublishTimeDifference_)
      {
        lastTime_ = initTime;

        SelfCollisionVisualizer_.publishDistances(pinocchioMapping_.getPinocchioJointVector(switched_model::joint_coordinate_t(currentState.tail(12))));
      }
    };

    void postSolverRun(const ocs2::PrimalSolution &primalSolution) override{};

  private:
    // const SwingTrajectoryPlanner* swingTrajectoryPlanner_;
    // TerrainPlaneVisualizer planeVisualizer_;
    GeometryInterfaceVisualization SelfCollisionVisualizer_;
    unitree::QuadrupedPinocchioMapping pinocchioMapping_;
    scalar_t lastTime_;
    scalar_t minPublishTimeDifference_;
  };

} // namespace legged
