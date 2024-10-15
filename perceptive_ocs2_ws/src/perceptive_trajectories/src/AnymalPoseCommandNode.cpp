/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-03-25 15:45:27
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-06 19:31:41
 * @FilePath: /OCS2_ws/src/perceptive_trajectories/src/AnymalPoseCommandNode.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
// #include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

#include "perceptive_trajectories/TargetTrajectoriesPublisher.h"

#include "perceptive_trajectories/PoseCommandToCostDesiredRos.h"

int main(int argc, char *argv[])
{


    std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name specified. Aborting.");
  }
  const std::string filename(programArgs[1]);
  const std::string robotName(programArgs[2]);


  // const std::string robotName = "anymal";

  // const std::string filename = [&]
  // {
  //   std::vector<std::string> programArgs{};
  //   ros::removeROSArgs(argc, argv, programArgs);
  //   if (programArgs.size() <= 1)
  //   {
  //     throw std::runtime_error("No task file specified. Aborting.");
  //   }
  //   return programArgs[1];
  // }();

  // ros node handle
  ::ros::init(argc, argv, robotName + "_mpc_pose_command_node");
  ::ros::NodeHandle nodeHandle;

  // PoseCommand To TargetTrajectories
  switched_model::PoseCommandToCostDesiredRos targetPoseCommand(nodeHandle, filename);
  auto goalToTargetTrajectoriesFun = [&](const ocs2::vector_t &goal, const ocs2::SystemObservation &observation)
  {
    return targetPoseCommand.goalToTargetTrajectories(goal, observation);
  };
  auto cmdVelToTargetTrajectoriesFun = [&](const ocs2::vector_t &cmdVel, const ocs2::SystemObservation &observation)
  {
    return targetPoseCommand.cmdVelToTargetTrajectories(cmdVel, observation);
  };
  // goalPose: [deltaX, deltaY, deltaZ, Roll, Pitch, deltaYaw]
  legged::TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, goalToTargetTrajectoriesFun, cmdVelToTargetTrajectoriesFun);
  
  ros::spin();

  // Successful exit
  return 0;
}
