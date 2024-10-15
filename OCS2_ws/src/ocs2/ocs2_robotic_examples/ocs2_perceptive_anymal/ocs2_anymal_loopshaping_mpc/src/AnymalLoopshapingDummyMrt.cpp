/*
 * @Author: NaCl
 * @Date: 2024-03-09 18:23:47
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-06 18:55:27
 * @FilePath: /OCS2_ws/src/ocs2/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/src/AnymalLoopshapingDummyMrt.cpp
 * @Description: 
 * 
 */
//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h>
#include <ros/init.h>

#include "ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h"

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
  ros::init(argc, argv, "anymal_loopshaping_mrt");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto anymalInterface = anymal::getAnymalLoopshapingInterface(urdfString, anymal::getConfigFolderLoopshaping(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePathLoopshaping(configName));
  quadrupedLoopshapingDummyNode(nodeHandle, *anymalInterface, mpcSettings.mrtDesiredFrequency_, mpcSettings.mpcDesiredFrequency_,robotName);

  return 0;
}
