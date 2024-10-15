/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-01 12:34:30
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-06 19:47:45
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_loopshaping_mpc/src/UnitreeLoopshapingDummyMrt.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h>
#include <ros/init.h>

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
  ros::init(argc, argv, "unitree_loopshaping_mrt");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto unitreeInterface = unitree::getUnitreeLoopshapingInterface(urdfString, unitree::getConfigFolderLoopshaping(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(unitree::getTaskFilePathLoopshaping(configName));
  quadrupedLoopshapingDummyNode(nodeHandle, *unitreeInterface, mpcSettings.mrtDesiredFrequency_, mpcSettings.mpcDesiredFrequency_,robotName);

  return 0;
}
