/*
 * @Author: NaCl
 * @Date: 2024-03-09 18:23:47
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-06 18:52:36
 * @FilePath: /OCS2_ws/src/ocs2/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_commands/src/AnymalMotionCommandNode.cpp
 * @Description: 
 * 
 */

#include <ros/package.h>

#include "ocs2_anymal_commands/MotionCommandController.h"
#include "ocs2_anymal_commands/MotionCommandDummy.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No robot name specified. Aborting.");
  }
  const std::string descriptionName(programArgs[1]);
  // const std::string configName(programArgs[2]);

  // const std::string robotName = descriptionName;
  const std::string robotName = "anymal";
  std::string motionFile = ros::package::getPath("ocs2_anymal_commands") + "/config/motions.info";
  std::cerr << "Loading motion file: " << motionFile << std::endl;

  const std::string controllerName = [&] {
    std::vector<std::string> programArgs{};
    ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1) {
      throw std::runtime_error("No operation mode specified. Aborting.");
    }
    return programArgs[1];
  }();

  ros::init(argc, argv, robotName + "_mpc_motion_command");
  ros::NodeHandle nodeHandle;

  std::unique_ptr<switched_model::MotionCommandInterface> motionCommandInterface;
  if (controllerName == "dummy") {
    motionCommandInterface.reset(new switched_model::MotionCommandDummy(nodeHandle, motionFile, robotName));
  } else {
    motionCommandInterface.reset(new switched_model::MotionCommandController(nodeHandle, motionFile, controllerName));
  }

  ros::Rate rate(10);
  while (ros::ok() && ros::master::check()) {
    motionCommandInterface->getKeyboardCommand();
    rate.sleep();
  }

  // Successful exit
  return 0;
}
