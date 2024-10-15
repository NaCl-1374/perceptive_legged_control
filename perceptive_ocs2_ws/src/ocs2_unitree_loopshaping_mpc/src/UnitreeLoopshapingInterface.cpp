//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_unitree_loopshaping_mpc/UnitreeLoopshapingInterface.h"

#include <ros/package.h>

#include <ocs2_unitree_mpc/UnitreeInterface.h>

namespace unitree
{

  std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getUnitreeLoopshapingInterface(const std::string &urdf,
                                                                                                            const std::string &configFolder)
  {
    // return getUnitreeLoopshapingInterface(urdf, switched_model::loadQuadrupedSettings(configFolder + "/task.info"),
    //                                      frameDeclarationFromFile(configFolder + "/frame_declaration.info"),
    //                                      ocs2::loopshaping_property_tree::load(configFolder + "/loopshaping.info"));

    // auto quadrupedInterface = getUnitreeInterface(urdf, std::move(switched_model::loadQuadrupedSettings(configFolder + "/task.info")),
    //                                               frameDeclarationFromFile(configFolder + "/frame_declaration.info"));
    std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition = ocs2::loopshaping_property_tree::load(configFolder + "/loopshaping.info");
    auto quadrupedInterface = getUnitreeInterface(urdf, configFolder);

    loopshapingDefinition->costMatrix() = quadrupedInterface->nominalCostApproximation().dfduu;
    loopshapingDefinition->print();

    return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
        new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), std::move(loopshapingDefinition)));
  }

  std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> getUnitreeLoopshapingInterface(
      const std::string &urdf, switched_model::QuadrupedInterface::Settings settings, const FrameDeclaration &frameDeclaration,
      std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition)
  {
    auto quadrupedInterface = getUnitreeInterface(urdf, std::move(settings), frameDeclaration);

    loopshapingDefinition->costMatrix() = quadrupedInterface->nominalCostApproximation().dfduu;
    loopshapingDefinition->print();

    return std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface>(
        new switched_model_loopshaping::QuadrupedLoopshapingInterface(std::move(quadrupedInterface), std::move(loopshapingDefinition)));
  }

  std::string getConfigFolderLoopshaping(const std::string &configName)
  {
    return ros::package::getPath("ocs2_unitree_loopshaping_mpc") + "/config/" + configName;
  }

  std::string getTaskFilePathLoopshaping(const std::string &configName)
  {
    return getConfigFolderLoopshaping(configName) + "/task.info";
  }

} // end of namespace unitree
