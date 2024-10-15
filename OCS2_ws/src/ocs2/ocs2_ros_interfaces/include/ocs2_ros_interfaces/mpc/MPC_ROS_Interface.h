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

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>

#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_msgs/reset.h>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#define PUBLISH_THREAD

namespace ocs2
{

  /**
   *该类使用ROS实现MPC通信接口。
   */
  class MPC_ROS_Interface
  {
  public:
     /**
     *构造函数。
     *主要初始化buffer和要发布的数据，要发布的数据类似于标准MPC_MRT_Interface中的激活解，初始化线程用来运行ros::publish函数
     *@param [in] mpc：要使用的底层 MPC 类。
     *@param [in] topicPrefix: 机器人的名字。
     */
    explicit MPC_ROS_Interface(MPC_BASE &mpc, std::string topicPrefix = "anonymousRobot");

    /**
     * Destructor.
     */
    virtual ~MPC_ROS_Interface();

  /**
     *将类重置为其实例化状态。
     * 重置所有，将mpc的参考轨迹设为initTargetTrajectories
     *@param [in] initTargetTrajectories：初始期望的成本轨迹。
     */
    void resetMpcNode(TargetTrajectories &&initTargetTrajectories);

    /**
     * Shutdowns the ROS node.
     */
    void shutdownNode();

    /**
     * Spins ROS.
     */
    void spin();

    /**
     *这是启动 MPC 运行所需的所有节点的主要例程，其中包括：
     *(1) MPC 策略发布者（反馈或前馈策略）。
     *(2) 获取当前测量状态调用MPC运行例程的观察订阅者。
     */
    void launchNodes(ros::NodeHandle &nodeHandle);

  protected:
    /**
     * Callback to reset MPC.
     *
     * @param req: Service request.
     * @param res: Service response.
     */
    bool resetMpcCallback(ocs2_msgs::reset::Request &req, ocs2_msgs::reset::Response &res);

    /**
     *创建 MPC 策略消息。
     * 将输入参数转为ros topic 
     *@param [in] primalSolution: MPC 的策略数据。
     *@param [in] commandData：MPC的命令数据。
     *@param [in] performanceIndices: 求解器的性能指标数据。
     *@return MPC 策略消息。
     */
    static ocs2_msgs::mpc_flattened_controller createMpcPolicyMsg(const PrimalSolution &primalSolution, const CommandData &commandData,
                                                                  const PerformanceIndex &performanceIndices);

    /**
     * Handles ROS publishing thread.
     */
    void publisherWorker();

    /**
     *从 MPC 对象更新缓冲区变量。此方法由 mpcObservationCallback() 自动调用
     *
     *@param [in] mpcInitObservation：用于运行 MPC 的观测量。
     */
    void copyToBuffer(const SystemObservation &mpcInitObservation);

    /**
     *接收当前观察的回调方法，调用 MPC 算法，
     *最后发布优化后的策略。
     *
     *@param [in] msg: 观察消息。
     */
    void mpcObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg);

  protected:
    /*
     * Variables
     */
    MPC_BASE &mpc_;

    std::string topicPrefix_;

    std::shared_ptr<ros::NodeHandle> nodeHandlerPtr_;

    // Publishers and subscribers
    ::ros::Subscriber mpcObservationSubscriber_;
    ::ros::Subscriber mpcTargetTrajectoriesSubscriber_;
    ::ros::Publisher mpcPolicyPublisher_;
    ::ros::ServiceServer mpcResetServiceServer_;

    std::unique_ptr<CommandData> bufferCommandPtr_;
    std::unique_ptr<CommandData> publisherCommandPtr_;
    std::unique_ptr<PrimalSolution> bufferPrimalSolutionPtr_;
    std::unique_ptr<PrimalSolution> publisherPrimalSolutionPtr_;
    std::unique_ptr<PerformanceIndex> bufferPerformanceIndicesPtr_;
    std::unique_ptr<PerformanceIndex> publisherPerformanceIndicesPtr_;

    mutable std::mutex bufferMutex_; // for policy variables with prefix (buffer*)

    // multi-threading for publishers
    std::atomic_bool terminateThread_{false};
    std::atomic_bool readyToPublish_{false};
    std::thread publisherWorker_;
    std::mutex publisherMutex_;
    std::condition_variable msgReady_;

    benchmark::RepeatedTimer mpcTimer_;

    // MPC reset
    std::mutex resetMutex_;
    std::atomic_bool resetRequestedEver_{false};
  };

} // namespace ocs2
