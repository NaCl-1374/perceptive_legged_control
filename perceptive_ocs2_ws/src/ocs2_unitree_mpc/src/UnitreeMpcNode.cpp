/*
 * @Author: NaCl
 * @Date: 2024-04-01 12:32:38
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-19 12:08:30
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_mpc/src/UnitreeMpcNode.cpp
 * @Description:
 *
 */
/*
 * UnitreeMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include <ros/init.h>

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedMpc.h>

#include "ocs2_unitree_mpc/UnitreeInterface.h"

int main(int argc, char *argv[])
{
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 4)
  {
    throw std::runtime_error("No robot name and config folder specified. Aborting.");
  }
  const std::string descriptionName(programArgs[1]);
  const std::string configName(programArgs[2]);
  const std::string robotName(programArgs[3]);

  // Initialize ros node
  ros::init(argc, argv, "unitree_mpc");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto unitreeInterface = unitree::getUnitreeInterface(urdfString, unitree::getConfigFolder(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(unitree::getTaskFilePath(configName));

  switch (unitreeInterface->modelSettings().algorithm_)
  {
  case switched_model::Algorithm::DDP:
  {
    const auto ddpSettings = ocs2::ddp::loadSettings(unitree::getTaskFilePath(configName));
    auto mpcPtr = getDdpMpc(*unitreeInterface, mpcSettings, ddpSettings);
    quadrupedMpcNode(nodeHandle, *unitreeInterface, std::move(mpcPtr), robotName);
    break;
  }
  case switched_model::Algorithm::SQP:
  {
    const auto sqpSettings = ocs2::sqp::loadSettings(unitree::getConfigFolder(configName) + "/multiple_shooting.info");
    auto mpcPtr = getSqpMpc(*unitreeInterface, mpcSettings, sqpSettings);
    quadrupedMpcNode(nodeHandle, *unitreeInterface, std::move(mpcPtr), robotName);
    break;
  }
  }

  return 0;
}
