/*
 * @Author: NaCl
 * @Date: 2024-04-01 12:32:38
 * @LastEditors: NaCl
 * @LastEditTime: 2024-05-18 18:52:21
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_mpc/src/UnitreeInterface.cpp
 * @Description:
 *
 */

//
// Created by rgrandia on 17.02.20.
//
#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "ocs2_unitree_mpc/UnitreeInterface.h"

#include <ros/package.h>

#include <ocs2_quadruped_interface/QuadrupedPointfootInterface.h>

// yxy add
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "ocs2_self_collision/PinocchioGeometryInterface.h"
#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

// #include <ocs2_legged_self_collishion/ocs2_legged_self_collishion.h>
#include <ocs2_legged_self_collishion/SelfCollisionConstraint.h>

namespace unitree
{
  using namespace ocs2;
  using namespace switched_model;

  class UnitreeSelfCollisionConstraint final : public SelfCollisionConstraint
  {
  public:
    UnitreeSelfCollisionConstraint(const FrameDeclaration &frameDeclaration, PinocchioInterface pinocchioInterface,
                                   PinocchioGeometryInterface pinocchioGeometryInterface,
                                   scalar_t minimumDistance)
        : SelfCollisionConstraint(frameDeclaration, std::move(pinocchioInterface), std::move(pinocchioGeometryInterface), minimumDistance) {}
    ~UnitreeSelfCollisionConstraint() override = default;
    UnitreeSelfCollisionConstraint(const UnitreeSelfCollisionConstraint &other) = default;
    UnitreeSelfCollisionConstraint *clone() const { return new UnitreeSelfCollisionConstraint(*this); }
  };

  std::unique_ptr<StateCost> getSelfCollisionConstraint(ocs2::PinocchioInterface pinocchioInterface,
                                                        const std::string &taskFile, const std::string &prefix,
                                                        bool verbose, const FrameDeclaration &frameDeclaration)
  {
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
    scalar_t mu = 1e-2;
    scalar_t delta = 1e-3;
    scalar_t minimumDistance = 0.0;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    if (verbose)
    {
      std::cerr << "\n #### SelfCollision Settings: ";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, mu, prefix + ".mu", verbose);
    loadData::loadPtreeValue(pt, delta, prefix + ".delta", verbose);
    loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", verbose);
    loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, verbose);
    loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, verbose);

    auto geometryInterfacePtr_ = std::make_unique<ocs2::PinocchioGeometryInterface>(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
    if (verbose)
    {
      std::cerr << " #### =============================================================================\n";
      // const size_t numCollisionPairs = geometryInterfacePtr_->getNumCollisionPairs();
      const size_t numCollisionPairs = geometryInterfacePtr_->getNumCollisionPairs();
      std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
    }
    // new test by vector
    // std::unique_ptr<StateConstraint> constraint;

    auto constraint = std::make_unique<UnitreeSelfCollisionConstraint>(frameDeclaration, pinocchioInterface, *geometryInterfacePtr_, minimumDistance);

    auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

    return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));

    // return std::make_unique<SelfCollisionAvoidanceCost>(
    //     frameDeclaration, pinocchioInterface, ocs2::RelaxedBarrierPenalty::Config{mu, delta}, *geometryInterfacePtr_, minimumDistance, "self_collision", "/tmp/ocs2", false, true);
  }

  std::unique_ptr<switched_model::QuadrupedInterface> getUnitreeInterface(const std::string &urdf, const std::string &taskFolder)
  {
    std::cerr << "Loading task file from: " << taskFolder << std::endl;

    auto UnitreeInterface = getUnitreeInterface(urdf, switched_model::loadQuadrupedSettings(taskFolder + "/task.info"),
                                                frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
    // auto selfCollisionCost =
    //     getSelfCollisionConstraint(std::move(ocs2::getPinocchioInterfaceFromUrdfString(urdf, pinocchio::JointModelFreeFlyer())), taskFolder + "/task.info",
    //                                "selfCollision", true,
    //                                frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
    auto selfCollisionCost =
        getSelfCollisionConstraint(std::move(ocs2::getPinocchioInterfaceFromUrdfString(urdf)), taskFolder + "/task.info",
                                   "selfCollision", true,
                                   frameDeclarationFromFile(taskFolder + "/frame_declaration.info"));
    UnitreeInterface->addProblemaCost("SelfCollision", std::move(selfCollisionCost));
    return UnitreeInterface;
  }

  std::unique_ptr<switched_model::QuadrupedInterface> getUnitreeInterface(const std::string &urdf,
                                                                          switched_model::QuadrupedInterface::Settings settings,
                                                                          const FrameDeclaration &frameDeclaration)
  {
    std::unique_ptr<switched_model::InverseKinematicsModelBase> invKin{nullptr};
    if (settings.modelSettings_.analyticalInverseKinematics_)
    {
      invKin = getUnitreeInverseKinematics(frameDeclaration, urdf);
    }
    auto kin = getUnitreeKinematics(frameDeclaration, urdf);
    auto kinAd = getUnitreeKinematicsAd(frameDeclaration, urdf);
    auto com = getUnitreeComModel(frameDeclaration, urdf);
    auto comAd = getUnitreeComModelAd(frameDeclaration, urdf);
    auto jointNames = getJointNames(frameDeclaration);
    auto baseName = frameDeclaration.root;

    return std::unique_ptr<switched_model::QuadrupedInterface>(new switched_model::QuadrupedPointfootInterface(
        *kin, *kinAd, *com, *comAd, invKin.get(), std::move(settings), std::move(jointNames), std::move(baseName)));
  }

  std::string getConfigFolder(const std::string &configName)
  {
    return ros::package::getPath("ocs2_unitree_mpc") + "/config/" + configName;
  }

  std::string getTaskFilePath(const std::string &configName)
  {
    return getConfigFolder(configName) + "/task.info";
  }

} // namespace unitree
