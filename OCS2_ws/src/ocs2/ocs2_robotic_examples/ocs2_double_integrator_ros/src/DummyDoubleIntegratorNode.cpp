/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_double_integrator/DoubleIntegratorInterface.h>
#include <ocs2_double_integrator/definitions.h>

#include "ocs2_double_integrator_ros/DoubleIntegratorDummyVisualization.h"

int main(int argc, char **argv)
{
  const std::string robotName = "double_integrator";

  // task file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1)
  {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  const std::string taskFile = ros::package::getPath("ocs2_double_integrator") + "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder = ros::package::getPath("ocs2_double_integrator") + "/auto_generated";
  ocs2::double_integrator::DoubleIntegratorInterface doubleIntegratorInterface(taskFile, libFolder);

  // MRT 负责安全访问MPC求解器的解
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&doubleIntegratorInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto doubleIntegratorDummyVisualization = std::make_shared<ocs2::double_integrator::DoubleIntegratorDummyVisualization>(nodeHandle);

  // Dummy loop 虚拟循环 接受mrt,MPC节点订阅观测值，发布MPC解。mrt订阅MPC解，发布观测值。
  // Dummy loop 则通过mrt订阅的MPC解，来计算之后的（状态，输入）
  ocs2::MRT_ROS_Dummy_Loop dummyDoubleIntegrator(mrt, doubleIntegratorInterface.mpcSettings().mrtDesiredFrequency_,
                                                 doubleIntegratorInterface.mpcSettings().mpcDesiredFrequency_);
  //  doubleIntegratorDummyVisualization是dummyObservers
  // 其有update函数在每次dummyLoop中被调用，其被传入mpc解算，下一时刻状态，mpc计算的状态。这里用来作可视化发布
  dummyDoubleIntegrator.subscribeObservers({doubleIntegratorDummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.time = 0.0;
  initObservation.state = doubleIntegratorInterface.getInitialState();
  initObservation.input = ocs2::vector_t::Zero(ocs2::double_integrator::INPUT_DIM);

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {doubleIntegratorInterface.getInitialTarget()},
                                                        {ocs2::vector_t::Zero(ocs2::double_integrator::INPUT_DIM)});

  // Run dummy (loops while ros is ok)
  dummyDoubleIntegrator.run(initObservation, initTargetTrajectories);

  return 0;
}
