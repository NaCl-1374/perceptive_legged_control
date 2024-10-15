/*
 * @Author: NaCl
 * @Date: 2024-03-09 18:23:47
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-06 18:56:11
 * @FilePath: /OCS2_ws/src/ocs2/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_mpc/src/AnymalDummyMRT.cpp
 * @Description: 
 * 
 */
/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>
#include <ros/init.h>

#include <ocs2_mpc/MPC_Settings.h>
#include "ocs2_anymal_mpc/AnymalInterface.h"

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
  ros::init(argc, argv, "anymal_mrt");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto anymalInterface = anymal::getAnymalInterface(urdfString, anymal::getConfigFolder(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePath(configName));
  quadrupedDummyNode(nodeHandle, *anymalInterface, &anymalInterface->getRollout(), mpcSettings.mrtDesiredFrequency_,
                     mpcSettings.mpcDesiredFrequency_,configName);

  return 0;
}
