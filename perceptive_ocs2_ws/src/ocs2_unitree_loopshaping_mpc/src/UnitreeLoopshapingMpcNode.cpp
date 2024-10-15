/*
 * @Author: NaCl
 * @Date: 2024-04-01 12:34:30
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-06 20:19:50
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_loopshaping_mpc/src/UnitreeLoopshapingMpcNode.cpp
 * @Description: 
 * 
 */
//
// Created by rgrandia on 13.02.20.
//

#include <ros/init.h>

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpc.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h>

#include "ocs2_unitree_loopshaping_mpc/UnitreeLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 4) {
    throw std::runtime_error("No robot name and config folder specified. Aborting.");
  }
  const std::string descriptionName(programArgs[1]);
  const std::string configName(programArgs[2]);
  const std::string robotName(programArgs[3]);
  // Initialize ros node
  ros::init(argc, argv, "unitree_loopshaping_mpc");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto unitreeInterface = unitree::getUnitreeLoopshapingInterface(urdfString, unitree::getConfigFolderLoopshaping(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(unitree::getTaskFilePathLoopshaping(configName));

  switch (unitreeInterface->modelSettings().algorithm_) {
    case switched_model::Algorithm::DDP: {
      const auto ddpSettings = ocs2::ddp::loadSettings(unitree::getTaskFilePathLoopshaping(configName));
      auto mpcPtr = getDdpMpc(*unitreeInterface, mpcSettings, ddpSettings);
      quadrupedLoopshapingMpcNode(nodeHandle, *unitreeInterface, std::move(mpcPtr),robotName);
      break;
    }
    case switched_model::Algorithm::SQP: {
      const auto sqpSettings = ocs2::sqp::loadSettings(unitree::getConfigFolderLoopshaping(configName) + "/multiple_shooting.info");
      auto mpcPtr = getSqpMpc(*unitreeInterface, mpcSettings, sqpSettings);
      quadrupedLoopshapingMpcNode(nodeHandle, *unitreeInterface, std::move(mpcPtr),robotName);
      break;
    }
  }

  return 0;
}
