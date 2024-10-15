/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include <ros/subscriber.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace ocs2
{

  /**
   *此类允许用户从命令行插入机器人命令。
   */
  class TargetTrajectoriesKeyboardPublisher final
  {
  public:
    using CommandLineToTargetTrajectories =
        std::function<TargetTrajectories(const vector_t &commadLineTarget, const SystemObservation &observation)>;//用户自定义的函数用来处理接收到的commad

    /**
     *构造器
     *
     *@param [in] nodeHandle: ROS 节点句柄。
     *@param [in] topicPrefix：TargetTrajectories 将发布在“topicPrefix_mpc_target”主题上。此外，最新的
     *预计将对“topicPrefix_mpc_observation”主题进行观察。
     *@param [in] targetCommandLimits：从命令行加载的命令的限制（出于安全目的）其长度被设置为状态维度。
     *@param [in] commandLineToTargetTrajectoriesFun：将命令行输入转换为 TargetTrajectories 的函数。
     */
    TargetTrajectoriesKeyboardPublisher(::ros::NodeHandle &nodeHandle, const std::string &topicPrefix,
                                        const scalar_array_t &targetCommandLimits,
                                        CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun);

    /** Gets the command vector size. */
    size_t targetCommandSize() const { return targetCommandLimits_.size(); }

    /**
     *发布命令行输入。如果输入的命令比预期的命令短
     *size(targetCommandSize)，该方法会将命令的其余部分设置为零。
     *
     *@param [in] commadMsg：要在屏幕上显示的消息。
     */
    void publishKeyboardCommand(const std::string &commadMsg = "Enter command, separated by space");

  private:
    /** Gets the target from command line. */
    vector_t getCommandLine();

    const vector_t targetCommandLimits_;
    CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun_;

    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

    ::ros::Subscriber observationSubscriber_;
    mutable std::mutex latestObservationMutex_;
    SystemObservation latestObservation_;
  };

} // namespace ocs2
