/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>
#include <ros/init.h>

#include <ocs2_mpc/MPC_Settings.h>
#include "ocs2_unitree_mpc/UnitreeInterface.h"

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
  ros::init(argc, argv, "unitree_mrt");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto unitreeInterface = unitree::getUnitreeInterface(urdfString, unitree::getConfigFolder(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(unitree::getTaskFilePath(configName));
  quadrupedDummyNode(nodeHandle, *unitreeInterface, &unitreeInterface->getRollout(), mpcSettings.mrtDesiredFrequency_,
                     mpcSettings.mpcDesiredFrequency_,robotName);

  return 0;
}
