/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-03-25 17:55:53
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-03-25 18:06:51
 * @FilePath: /leg_perceptive_ws/src/perceptive_trajectories/include/perceptive_trajectories/ReferenceExtrapolation.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by rgrandia on 02.03.20.
//

#pragma once

#include <grid_map_core/GridMap.hpp>

#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model
{

  struct BaseReferenceHorizon
  {
    double dt;
    size_t N;
  };

  struct BaseReferenceState
  {
    double t0;
    Eigen::Vector3d positionInWorld;
    Eigen::Vector3d eulerXyz;
  };

  struct BaseReferenceCommand
  {
    double headingVelocity;
    double lateralVelocity;
    double yawRate;
    double baseHeight;
  };

  struct WorldReferenceCommand
  {
    double xVelocity;
    double yVelocity;
    double yawRate;
    double baseHeight;
  };

  struct Base2dReferenceTrajectory
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<double> time;
    std::vector<double> yaw;
    std::vector<Eigen::Vector2d> positionInWorld;
  };

  struct BaseReferenceTrajectory
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<double> time;
    std::vector<Eigen::Vector3d> eulerXyz;
    std::vector<Eigen::Vector3d> positionInWorld;
    std::vector<Eigen::Vector3d> linearVelocityInWorld;
    std::vector<Eigen::Vector3d> angularVelocityInWorld;
  };

  Eigen::Vector2d velocityCommand2dInWorld(double headingVelocity, double lateralVelocity, double yaw);

  Base2dReferenceTrajectory generate2DExtrapolatedBaseReference(const BaseReferenceHorizon &horizon, const BaseReferenceState &initialState,
                                                                const BaseReferenceCommand &command);

  Base2dReferenceTrajectory generate2DExtrapolatedBaseReference(const BaseReferenceHorizon &horizon, const BaseReferenceState &initialState,
                                                                const WorldReferenceCommand &command);

  BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon &horizon, const BaseReferenceState &initialState,
                                                            const BaseReferenceCommand &command,
                                                            const switched_model::TerrainPlane &projectedHeadingFrame);

  BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon &horizon, const BaseReferenceState &initialState,
                                                            const BaseReferenceCommand &command, const grid_map::GridMap &gridMap,
                                                            double nominalStanceWidthInHeading, double nominalStanceWidthLateral);

  BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon &horizon, const BaseReferenceState &initialState,
                                                            const WorldReferenceCommand &command,
                                                            const switched_model::TerrainPlane &projectedHeadingFrame);

  BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon &horizon, const BaseReferenceState &initialState,
                                                            const WorldReferenceCommand &command, const grid_map::GridMap &gridMap,
                                                            double nominalStanceWidthInHeading, double nominalStanceWidthLateral);

} // namespace switched_model
