//
// Created by qiayuan on 22-12-27.
//

#include "legged_perceptive_interface/constraint/FootCollisionConstraint.h"
#include "legged_perceptive_interface/constraint/SphereSdfConstraint.h"

#include "legged_perceptive_interface/ConvexRegionSelector.h"
#include "legged_perceptive_interface/PerceptiveLeggedInterface.h"
#include "legged_perceptive_interface/PerceptiveLeggedPrecomputation.h"
#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <memory>

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include "legged_perceptive_interface/constraint/FootPlacementConstraintCBF.h"
#include "legged_perceptive_interface/constraint/SwingFootPlacementConstraintCBF.h"
namespace legged {

void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile, bool verbose) {
  planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

  double width{5.0}, height{5.0};
  convex_plane_decomposition::PlanarRegion plannerRegion;
  plannerRegion.transformPlaneToWorld.setIdentity();
  plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-height / 2, -width / 2, +height / 2, width / 2);
  convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, +width / 2));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, +width / 2));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, -width / 2));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, -width / 2));
  plannerRegion.boundaryWithInset.boundary = boundary;
  convex_plane_decomposition::CgalPolygonWithHoles2d insets;
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, +width / 2 - 0.01));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, +width / 2 - 0.01));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, -width / 2 + 0.01));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, -width / 2 + 0.01));
  plannerRegion.boundaryWithInset.insets.push_back(insets);
  planarTerrainPtr_->planarRegions.push_back(plannerRegion);

  std::string layer = "elevation_before_postprocess";
  planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);
  planarTerrainPtr_->gridMap.add(layer, 0);
  planarTerrainPtr_->gridMap.add("smooth_planar", 0);
  signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>(planarTerrainPtr_->gridMap, layer, -0.1, 0.1);
  
  RelaxedBarrierPenalty::Config placemenCBFPenaltyConfig;
  RelaxedBarrierPenalty::Config placemenPenaltyConfig;
  scalar_t cbfLambda =1;
  bool enable_footpalcement=true;
  bool enable_CBF=true;

  RelaxedBarrierPenalty::Config swingPlacemenCBFPenaltyConfig;
  RelaxedBarrierPenalty::Config swingPlacemenPenaltyConfig;
  scalar_t swingcbfLambda =1;
  bool enableSwingCBF=true;

  bool enableFootCollision=true;
  bool enableSphereSdfConstraint=false;
  RelaxedBarrierPenalty::Config footCollisionPenaltyConfig, sphereSdfConstraintPenaltyConfig;

  std::tie(enable_footpalcement,enable_CBF,cbfLambda,placemenCBFPenaltyConfig,placemenPenaltyConfig)=loadFootPlacementSettings(taskFile,verbose);
  
  std::tie(enableSwingCBF,swingcbfLambda,swingPlacemenCBFPenaltyConfig,swingPlacemenPenaltyConfig)=loadSwingFootPlacementSettings(taskFile,verbose);


  std::tie(enableFootCollision,enableSphereSdfConstraint,footCollisionPenaltyConfig,sphereSdfConstraintPenaltyConfig)=loadCollisionSettings(taskFile,verbose);

  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings().contactNames3DoF[i];
    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

    // std::unique_ptr<PenaltyBase> placementPenalty(new RelaxedBarrierPenalty(placemenPenaltyConfig));
    // std::unique_ptr<PenaltyBase> placementCBFPenalty(new RelaxedBarrierPenalty(placemenCBFPenaltyConfig));

    // std::unique_ptr<PenaltyBase> collisionPenalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));

    // For foot placement
    if (enable_footpalcement) {
      std::cerr << "\n #### FootPlacement state Constraint add to problem: ";

      std::unique_ptr<FootPlacementConstraint> footPlacementConstraint(
          new FootPlacementConstraint(*referenceManagerPtr_, *eeKinematicsPtr,
                                      i, numVertices_));
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_footPlacement",
          std::unique_ptr<StateCost>(new StateSoftConstraint(
              std::move(footPlacementConstraint),
              std::make_unique<RelaxedBarrierPenalty>(placemenPenaltyConfig))));
      std::cerr
          << " #### "
             "============================================================="
             "================\n";
    }
    if (enable_CBF) {
      // For foot placementCBF
      std::cerr << "\n #### FootPlacement cbf Constraint add to problem: ";
      std::unique_ptr<FootPlacementConstraintCBF> footPlacementConstraintCBF(
          new FootPlacementConstraintCBF(*referenceManagerPtr_,
                                         *eeKinematicsPtr, i, numVertices_,
                                         cbfLambda));
      problemPtr_->softConstraintPtr->add(
          footName + "_footPlacementCBF",
          std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(
              std::move(footPlacementConstraintCBF),
              std::make_unique<RelaxedBarrierPenalty>(
                  placemenCBFPenaltyConfig))));
      std::cerr
          << " #### "
             "============================================================="
             "================\n";
    }
    if (enableSwingCBF) {
        // For foot placementCBF
        std::cerr << "\n #### Swing FootPlacement cbf Constraint add to problem: ";
        std::unique_ptr<SwingFootPlacementConstraintCBF> swingFootPlacementConstraintCBF(
            new SwingFootPlacementConstraintCBF(*referenceManagerPtr_,
                                                *eeKinematicsPtr, i, numVertices_,
                                                swingcbfLambda,
                                                std::make_unique<RelaxedBarrierPenalty>(swingPlacemenPenaltyConfig)));
        problemPtr_->softConstraintPtr->add(
            footName + "_swingFootPlacementCBF",
            std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(
                std::move(swingFootPlacementConstraintCBF),
                std::make_unique<RelaxedBarrierPenalty>(
                    swingPlacemenCBFPenaltyConfig))));
        std::cerr
            << " #### "
              "============================================================="
              "================\n";
    }
    // For foot Collision
    if (enableFootCollision) {
      std::cerr << "\n #### foot Collision Constraint add to problem: ";
      std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
          new FootCollisionConstraint(*referenceManagerPtr_, *eeKinematicsPtr,
                                      signedDistanceFieldPtr_, i, 0.03));
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_footCollision",
          std::unique_ptr<StateCost>(
              new StateSoftConstraint(std::move(footCollisionConstraint),
                                      std::make_unique<RelaxedBarrierPenalty>(
                                          footCollisionPenaltyConfig))));
      std::cerr
          << " #### "
             "============================================================="
             "================\n";
    }
  }

  // For collision avoidance
 

    scalar_t thighExcess = 0.025;
    scalar_t calfExcess = 0.02;

    std::vector<std::string> collisionLinks = {"LF_calf", "RF_calf", "LH_calf",
                                               "RH_calf"};
    const std::vector<scalar_t>& maxExcesses = {calfExcess, calfExcess,
                                                calfExcess, calfExcess};

    pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(
        *pinocchioInterfacePtr_, collisionLinks, maxExcesses, 0.6);

    CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);
    auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(
        *pinocchioSphereInterfacePtr_, pinocchioMapping);
    if (enableSphereSdfConstraint) {
      std::cerr << "\n #### SphereSdf Constraint add to problem: ";
      std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(
          new SphereSdfConstraint(*sphereKinematicsPtr,
                                  signedDistanceFieldPtr_));

      //  std::unique_ptr<PenaltyBase> penalty(new
      //  RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-3, 1e-3)));
      problemPtr_->stateSoftConstraintPtr->add(
          "sdfConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(
                               std::move(sphereSdfConstraint),
                               std::make_unique<RelaxedBarrierPenalty>(
                                   sphereSdfConstraintPenaltyConfig))));
      std::cerr
          << " #### "
             "============================================================="
             "================\n";
    }
}

void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                      const std::string& referenceFile, bool verbose) {
  auto swingTrajectoryPlanner =
      std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({modelSettings_.contactNames3DoF}, "ALL_FOOT");
  auto convexRegionSelector =
      std::make_unique<ConvexRegionSelector>(centroidalModelInfo_, planarTerrainPtr_, *eeKinematicsPtr, numVertices_);

  scalar_t comHeight = 0;
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  referenceManagerPtr_.reset(new PerceptiveLeggedReferenceManager(centroidalModelInfo_, loadGaitSchedule(referenceFile, verbose),
                                                                  std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
                                                                  *eeKinematicsPtr, comHeight));
}

void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/, const std::string& /*urdfFile*/,
                                                    const std::string& /*referenceFile*/, bool /*verbose*/) {
  problemPtr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_,
      *dynamic_cast<PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_).getConvexRegionSelectorPtr());
}

std::tuple<bool, bool, scalar_t, RelaxedBarrierPenalty::Config,
           RelaxedBarrierPenalty::Config>
PerceptiveLeggedInterface::loadFootPlacementSettings(
    const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlacement.";

  bool enable = 1.0;
  bool enable_CBF = 1.0;

  scalar_t cbf_Lambda = 1.0;

  RelaxedBarrierPenalty::Config barrierPenaltyConfigCBF;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;

  if (verbose) {
    std::cerr << "\n #### FootPlacement Settings: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, enable, prefix + "enable", verbose);
  loadData::loadPtreeValue(pt, enable_CBF, prefix + "enableCBF", verbose);

  loadData::loadPtreeValue(pt, cbf_Lambda, prefix + "cbfLambda", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfigCBF.mu,
                           prefix + "cbfPenaltyMuParam", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfigCBF.delta,
                           prefix + "cbfPenaltyDeltaParam", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu,
                           prefix + "statePenaltyMuParam", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta,
                           prefix + "statePenaltyDeltaParam", verbose);
  if (verbose) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  return {enable, enable_CBF, cbf_Lambda, barrierPenaltyConfigCBF,
          barrierPenaltyConfig};
}

std::tuple<bool, bool, RelaxedBarrierPenalty::Config,
           RelaxedBarrierPenalty::Config>
PerceptiveLeggedInterface::loadCollisionSettings(const std::string& taskFile,
                                                 bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "CollisionParam.";

  bool enablefootCollision = false;
  bool enableSphereSdfConstraint = false;

  RelaxedBarrierPenalty::Config footCollisionPenaltyConfig;
  RelaxedBarrierPenalty::Config sphereSdfConstraintPenaltyConfig;

  if (verbose) {
    std::cerr << "\n #### CollisionParam Settings: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, enablefootCollision,
                           prefix + "enablefootCollision", verbose);

  loadData::loadPtreeValue(pt, enableSphereSdfConstraint,
                           prefix + "enableSphereSdfConstraint", verbose);
  loadData::loadPtreeValue(pt, footCollisionPenaltyConfig.mu,
                           prefix + "footCollisionPenaltyMu", verbose);
  loadData::loadPtreeValue(pt, footCollisionPenaltyConfig.delta,
                           prefix + "footCollisionPenaltyDelta", verbose);
  loadData::loadPtreeValue(pt, sphereSdfConstraintPenaltyConfig.mu,
                           prefix + "SphereSdfConstraintPenaltyMu", verbose);
  loadData::loadPtreeValue(pt, sphereSdfConstraintPenaltyConfig.delta,
                           prefix + "SphereSdfConstraintPenaltyDelta", verbose);
  if (verbose) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  return {enablefootCollision, enableSphereSdfConstraint,
          footCollisionPenaltyConfig, sphereSdfConstraintPenaltyConfig};
}

std::tuple<bool, scalar_t, RelaxedBarrierPenalty::Config,
           RelaxedBarrierPenalty::Config>
PerceptiveLeggedInterface::loadSwingFootPlacementSettings(
    const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlacement.";

  bool enable_CBF = 1.0;

  scalar_t cbf_Lambda = 1.0;

  RelaxedBarrierPenalty::Config barrierPenaltyConfigCBF;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;

  if (verbose) {
    std::cerr << "\n #### FootPlacement Settings: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, enable_CBF, prefix + "enableSwingCBF", verbose);

  loadData::loadPtreeValue(pt, cbf_Lambda, prefix + "swingcbfLambda", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfigCBF.mu,
                           prefix + "swingcbfPenaltyMuParam", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfigCBF.delta,
                           prefix + "swingcbfPenaltyDeltaParam", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu,
                           prefix + "swingPenaltyMuParam", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta,
                           prefix + "swingPenaltyDeltaParam", verbose);
  if (verbose) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  return {enable_CBF, cbf_Lambda, barrierPenaltyConfigCBF,
          barrierPenaltyConfig};
}

}  // namespace legged
