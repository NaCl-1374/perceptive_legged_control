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

#pragma once

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>

// MPC messages
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/reset.h>

#include <ocs2_mpc/MRT_BASE.h>

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

#define PUBLISH_THREAD

namespace ocs2
{

  /**
   *该类使用ROS实现MRT（Model Reference Tracking）通信接口。
   */
  class MRT_ROS_Interface : public MRT_BASE
  {
  public:
    /**
     *构造器
     *
     *@param [in] topicPrefix：前缀定义名称：观察的发布主题“topicPrefix_mpc_observation”，
     *策略的接收主题“topicPrefix_mpc_policy”，以及 MPC 重置服务“topicPrefix_mpc_reset”。
     *@param [in] mrtTransportHints: ROS 传输协议。
     */
    explicit MRT_ROS_Interface(std::string topicPrefix = "anonymousRobot",
                               ::ros::TransportHints mrtTransportHints = ::ros::TransportHints().tcpNoDelay());

    /**
     * Destructor
     */
    ~MRT_ROS_Interface() override;
    /**
     * restet mpc 通过ros 服务请求进行，和MPC_ROS_Interface进行交互
     */
    void resetMpcNode(const TargetTrajectories &initTargetTrajectories) override;

    /**
     * Shut down the ROS nodes.
     */
    void shutdownNodes();

    /**
     * Shut down publisher
     */
    void shutdownPublisher();

    /**
     * spin the MRT callback queue
     */
    void spinMRT();

    /**
     *启动 ROS 发布者和订阅者以与 MPC 节点通信。
     *@param 节点句柄
     */
    void launchNodes(::ros::NodeHandle &nodeHandle);
    /**
     *  通过ros topic 请求观测更新，和MPC_ROS_Interface进行交互,mpc在触发该消息回调函数时会计算一次mpc
     */
    void setCurrentObservation(const SystemObservation &currentObservation) override;

  private:
    /**
     *接收 MPC 策略和模式序列的回调方法。
     *它只更新带有后缀 (*Buffer_) 变量的策略变量。
     *把mpc解存入MRT缓冲区
     *
     *@param [in] msg: 指向消息的常量指针
     */
    void mpcPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr &msg);

    /**
     *读取 MPC 策略消息的辅助函数。
     *将接受到的msg解析成commandData，primalSolution，performanceIndices
     *@param [in] msg: 指向消息的常量指针
     *@param [out] commandData: MPC 命令数据
     *@param [out] primalSolution: MPC 策略数据
     *@param [out] performanceIndices: MPC 性能指标数据
     */
    static void readPolicyMsg(const ocs2_msgs::mpc_flattened_controller &msg, CommandData &commandData, PrimalSolution &primalSolution,
                              PerformanceIndex &performanceIndices);

    /**
     *发送当前状态并检查新的 MPC 更新的线程函数。
     */
    void publisherWorkerThread();

  private:
    std::string topicPrefix_;

    // Publishers and subscribers
    ::ros::Publisher mpcObservationPublisher_;
    ::ros::Subscriber mpcPolicySubscriber_;
    ::ros::ServiceClient mpcResetServiceClient_;

    // ROS messages
    ocs2_msgs::mpc_observation mpcObservationMsg_;
    ocs2_msgs::mpc_observation mpcObservationMsgBuffer_;

    ::ros::CallbackQueue mrtCallbackQueue_;
    ::ros::TransportHints mrtTransportHints_;

    // Multi-threading for publishers
    bool terminateThread_;
    bool readyToPublish_;
    std::thread publisherWorker_;
    std::mutex publisherMutex_;
    std::condition_variable msgReady_;
  };

} // namespace ocs2
