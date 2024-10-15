//
// Created by qiayuan on 23-1-3.
//
#pragma once

#include <memory>

#include "legged_perceptive_interface/ConvexRegionSelector.h"

#include <legged_interface/SwitchedModelReferenceManager.h>
#include <tuple>
#include <ocs2_legged_robot/gait/LegLogic.h>

namespace legged {

using namespace ocs2;
using namespace legged_robot;

class PerceptiveLeggedReferenceManager : public SwitchedModelReferenceManager {
 public:
  PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                   std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                   std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics, scalar_t comHeight);

  const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }

  contact_flag_t getFootPlacementFlags(scalar_t time) const;
  contact_flag_t getSwingFootPlacementFlags(scalar_t time) const;

  feet_array_t<LegPhase> ContactPhasePerLeg(scalar_t time) const;
  feet_array_t<LegPhase> SwingPhasePerLeg(scalar_t time) const;

  feet_array_t<scalar_t> TimeOfNextLiftOff(scalar_t time) const;
  feet_array_t<scalar_t> TimeOfNextTouchDown(scalar_t time) const;

  std::pair<feet_array_t<std::vector<ContactTiming>>,feet_array_t<std::vector<SwingTiming>>> extractTimings()const;

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  virtual void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState, ModeSchedule& modeSchedule, const grid_map::GridMap& map);

  void modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                         const std::vector<bool>& contactFlagStocks,
                         std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections);

  std::tuple<scalar_array_t, scalar_array_t, scalar_array_t> getHeights(const std::vector<bool>& contactFlagStocks,
                                                       const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections, const grid_map::GridMap& map);

  const CentroidalModelInfo info_;
  feet_array_t<vector3_t> lastLiftoffPos_;

  std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

  scalar_t comHeight_;
};

}  // namespace legged
