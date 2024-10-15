/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-03-25 11:51:47
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-03-25 17:57:18
 * @FilePath: /leg_perceptive_ws/src/perceptive_trajectories/include/perceptive_trajectories/TargetTrajectoriesPublisher.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

// PlanarTerrain
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
// Pose Adjust
// #include <ocs2_anymal_commands/ReferenceExtrapolation.h>
#include "perceptive_trajectories/ReferenceExtrapolation.h"

namespace switched_model
{
  using namespace ocs2;

  class PoseCommandToCostDesiredRos

  {
  public:
    PoseCommandToCostDesiredRos(::ros::NodeHandle &nh, const std::string &configFile);

    void setNormData(scalar_t targetDisplacementVelocity, scalar_t targetRotationVelocity, scalar_t comHeight, switched_model::joint_coordinate_t defaultJointState, scalar_t time_to_target)
    {
      targetDisplacementVelocity_ = targetDisplacementVelocity;
      targetRotationVelocity_ = targetRotationVelocity;
      comHeight_ = comHeight;
      defaultJointState_ = defaultJointState;
      time_to_target_ = time_to_target;
    }
    TargetTrajectories goalToTargetTrajectories(const vector_t &goal, const SystemObservation &observation) ;
    TargetTrajectories cmdVelToTargetTrajectories(const vector_t &cmdVel, const SystemObservation &observation) ;

  private:
    scalar_t desiredTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const;

    std::mutex terrainMutex;
    std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr;

    scalar_t targetDisplacementVelocity_;
    scalar_t targetRotationVelocity_;
    scalar_t comHeight_;
    joint_coordinate_t defaultJointState_;
    scalar_t time_to_target_;

    ros::Subscriber terrainSubscriber_;

    const double nominalStanceWidthInHeading_ = 2.0 * (std::abs(0.2) + 0.15);
    const double nominalStanceWidthLateral_ = 2.0 * (std::abs(0.1) + 0.10);
  };

} // namespace legged
