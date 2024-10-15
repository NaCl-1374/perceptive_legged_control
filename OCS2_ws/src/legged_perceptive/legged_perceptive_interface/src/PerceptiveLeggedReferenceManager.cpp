//
// Created by qiayuan on 23-1-3.
//
#include <utility>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"
#include <grid_map_core/iterators/LineIterator.hpp>
namespace legged {

PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                   std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                   std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   scalar_t comHeight)
    : info_(std::move(info)),
      SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      comHeight_(comHeight) {}

void PerceptiveLeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  TargetTrajectories newTargetTrajectories;
  const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
  int nodeNum = 11;
  for (size_t i = 0; i < nodeNum; ++i) {
    scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
    vector_t state = targetTrajectories.getDesiredState(time);
    vector_t input = targetTrajectories.getDesiredState(time);

    vector_t pos = centroidal_model::getBasePose(state, info_).head(3);

    // Base Orientation
    scalar_t step = 0.3;
    grid_map::Vector3 normalVector;
    normalVector(0) = (map.atPosition("smooth_planar", pos + grid_map::Position(-step, 0)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(step, 0))) /
                      (2 * step);
    normalVector(1) = (map.atPosition("smooth_planar", pos + grid_map::Position(0, -step)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(0, step))) /
                      (2 * step);
    normalVector(2) = 1;
    normalVector.normalize();
    matrix3_t R;
    scalar_t z = centroidal_model::getBasePose(state, info_)(3);
    R << cos(z), -sin(z), 0,  // clang-format off
             sin(z), cos(z), 0,
             0, 0, 1;  // clang-format on
    vector_t v = R.transpose() * normalVector;
    centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

    // Base Z Position
    centroidal_model::getBasePose(state, info_)(2) =
        map.atPosition("smooth_planar", pos) + comHeight_ ;/// cos(centroidal_model::getBasePose(state, info_)(4));

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);
  }
  targetTrajectories = newTargetTrajectories;

  // Footstep
  convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

  // Swing trajectory
  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule,map);
}


void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                    ModeSchedule& modeSchedule,const grid_map::GridMap& map) {
  const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence,swingHeightSequence;

  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);

    auto projections = convexRegionSelectorPtr_->getProjections(leg);
    modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

    scalar_array_t liftOffHeights, touchDownHeights,swingHeights;
    std::tie(liftOffHeights, touchDownHeights,swingHeights) = getHeights(contactFlagStocks[leg], projections,map);
    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
    swingHeightSequence[leg]=swingHeights;
  }
  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence,swingHeightSequence);
}
void PerceptiveLeggedReferenceManager::modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                                                         const std::vector<bool>& contactFlagStocks,
                                                         std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections) {
  if (contactFlagStocks[initIndex]) {//处理连续接触事件
    lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
    lastLiftoffPos_[leg].z() -= 0.02;
    for (int i = initIndex; i < projections.size(); ++i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
    for (int i = initIndex; i >= 0; --i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
  }
  if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg]) {//小于初始支撑结束时间的投影点全部设为该接触点
    for (int i = initIndex; i >= 0; --i) {
      if (contactFlagStocks[i]) {
        projections[i].positionInWorld = lastLiftoffPos_[leg];
      }
      if (!contactFlagStocks[i] && !contactFlagStocks[i + 1]) {
        break;
      }
    }
  }
    //  for (int i = 0; i < projections.size(); ++i) {
    //    if (leg == 1) std::cerr << std::setprecision(3) << projections[i].positionInWorld.z() << "\t";
    //  }
    //  std::cerr << std::endl;
}

  //yxy add
scalar_t findMaxValueAlongLine(const grid_map::GridMap& map, std::string layer,
                               const grid_map::Position& startPoint,
                               const grid_map::Position& endPoint) {
                                  // 边界检查，确保起点和终点在地图范围内
  if (!map.isInside(startPoint) || !map.isInside(endPoint)) {
    // 处理越界的情况，可能抛出异常或返回一个默认值
    throw std::out_of_range("Start or end point is outside the map.");
  }
  // 获取起点和终点的高度值
  scalar_t startValue = map.atPosition(layer, startPoint);
  scalar_t endValue = map.atPosition(layer, endPoint);

  // 初始化最大值
  scalar_t maxValue = std::max(startValue, endValue);
  // 沿连线查找最大值
  grid_map::Index lineIterator;
  for (grid_map::LineIterator iterator(map, startPoint, endPoint);
       !iterator.isPastEnd(); ++iterator) {
    lineIterator = *iterator;

    // 获取当前单元格的值
    scalar_t cellValue = map.at(layer, lineIterator);

    // 更新最大值
    maxValue = std::max(maxValue, cellValue);
  }

  return maxValue;
}

std::tuple<scalar_array_t, scalar_array_t, scalar_array_t>  PerceptiveLeggedReferenceManager::getHeights(
    const std::vector<bool>& contactFlagStocks, const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections, const grid_map::GridMap& map) {
  std::vector<scalar_t> liftOffHeights, touchDownHeights,swingHeights;//yxy add
  const size_t numPhases = projections.size();
  liftOffHeights.clear();
  liftOffHeights.resize(numPhases);
  touchDownHeights.clear();
  touchDownHeights.resize(numPhases);
  //yxy add
  swingHeights.clear();
  swingHeights.resize(numPhases);
  std::vector<Eigen::Vector3d> liftOffProjections(numPhases, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> touchDownProjections(numPhases, Eigen::Vector3d::Zero());

  for (size_t i = 1; i < numPhases; ++i) {
    if (!contactFlagStocks[i]) {
    liftOffHeights[i] = contactFlagStocks[i - 1] ? projections[i - 1].positionInWorld.z() : liftOffHeights[i - 1];
    liftOffProjections[i]=contactFlagStocks[i - 1] ? projections[i - 1].positionInWorld : liftOffProjections[i - 1];
    }
  }
  for (int i = numPhases - 2; i >= 0; --i) {
    if (!contactFlagStocks[i]) {
      touchDownHeights[i] = contactFlagStocks[i + 1] ? projections[i + 1].positionInWorld.z() : touchDownHeights[i + 1];
      touchDownProjections[i] = contactFlagStocks[i + 1] ? projections[i + 1].positionInWorld : touchDownProjections[i + 1];

    }
  }
  
  //yxy add
  for (size_t i = 0; i < numPhases; ++i) {
    if (!contactFlagStocks[i]) {
      auto max_height = findMaxValueAlongLine(map, "elevation", liftOffProjections[i].head(2), touchDownProjections[i].head(2));
      swingHeights[i] =max_height*1.05;
      // std::cout<<max_height<<"\n";
    }
  }

  //  for (int i = 0; i < numPhases; ++i) {
  //    std::cerr << std::setprecision(3) << liftOffHeights[i] << "\t";
  //  }
  //  std::cerr << std::endl;
  //  for (int i = 0; i < numPhases; ++i) {
  //    std::cerr << std::setprecision(3) << contactFlagStocks[i] << "\t";
  //  }
  //  std::cerr << std::endl;

  return {liftOffHeights, touchDownHeights,swingHeights};
}
//若mpc开始计算时是接触状态，则这条腿的约束不激活，防止造成腿不变动，直到在之后的预测中的接触状态才会激活
contact_flag_t PerceptiveLeggedReferenceManager::getFootPlacementFlags(scalar_t time) const {
  contact_flag_t flag;
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
  for (int i = 0; i < flag.size(); ++i) {
    flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}
contact_flag_t PerceptiveLeggedReferenceManager::getSwingFootPlacementFlags(scalar_t time) const {
  contact_flag_t flag;
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
  for (int i = 0; i < flag.size(); ++i) {

    flag[i] = !getContactFlags(time)[i] && time >= finalTime[i] && !getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}

feet_array_t<LegPhase> PerceptiveLeggedReferenceManager::ContactPhasePerLeg(
    scalar_t time) const {
  return getContactPhasePerLeg(time, getModeSchedule());
}
feet_array_t<LegPhase> PerceptiveLeggedReferenceManager::SwingPhasePerLeg(
    scalar_t time) const {
  return getSwingPhasePerLeg(time, getModeSchedule());
}

std::pair<feet_array_t<std::vector<ContactTiming>>,
          feet_array_t<std::vector<SwingTiming>>>
PerceptiveLeggedReferenceManager::extractTimings() const {
  // auto ans=extractContactFlags(standing_trot.modeSequence);
  feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;
  feet_array_t<std::vector<SwingTiming>> swingTimingsPerLeg;

  // Convert mode sequence to a contact flag vector per leg
  const auto contactSequencePerLeg =
      extractContactFlags(getModeSchedule().modeSequence);

  // Extract timings per leg
  for (size_t leg = 0; leg < contactTimingsPerLeg.size(); ++leg) {
    contactTimingsPerLeg[leg] = extractContactTimings(
        getModeSchedule().eventTimes, contactSequencePerLeg[leg]);
    swingTimingsPerLeg[leg] = extractSwingTimings(getModeSchedule().eventTimes,
                                                  contactSequencePerLeg[leg]);
  }

  return {contactTimingsPerLeg, swingTimingsPerLeg};
}

//如果一直接触，则返回nan
feet_array_t<scalar_t> PerceptiveLeggedReferenceManager::TimeOfNextLiftOff(
    scalar_t time) const {
  feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;
  feet_array_t<std::vector<SwingTiming>> swingTimingsPerLeg;
  std::tie(contactTimingsPerLeg, swingTimingsPerLeg) = extractTimings();
  feet_array_t<scalar_t> TimeOfNextLiftOff;
  for (size_t leg = 0; leg < contactTimingsPerLeg.size(); ++leg) {
    TimeOfNextLiftOff[leg] =
        getTimeOfNextLiftOff(time, contactTimingsPerLeg[leg]);
  }
  return TimeOfNextLiftOff;
}
//如果一直摆动，则返回nan
feet_array_t<scalar_t> PerceptiveLeggedReferenceManager::TimeOfNextTouchDown(
    scalar_t time) const {
  feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;
  feet_array_t<std::vector<SwingTiming>> swingTimingsPerLeg;
  std::tie(contactTimingsPerLeg, swingTimingsPerLeg) = extractTimings();
  feet_array_t<scalar_t> TimeOfNextTouchDown;
  for (size_t leg = 0; leg < contactTimingsPerLeg.size(); ++leg) {
    TimeOfNextTouchDown[leg] =
        getTimeOfNextTouchDown(time, contactTimingsPerLeg[leg]);
  }
  return TimeOfNextTouchDown;
}

}  // namespace legged
