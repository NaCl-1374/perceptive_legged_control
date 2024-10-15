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

#include "ocs2_ros_interfaces/mrt/DummyObserver.h"
#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"

namespace ocs2
{

  /**
*这个类实现了一个循环来测试使用 ROS 的 MPC-MRT 通信接口。
*/
  class MRT_ROS_Dummy_Loop
  {
  public:
    /**
*构造函数。
*
*@param [in] mrt：要使用的底层 MRT 类。如果 MRT 包含rollout对象，则虚拟对象将使用 MRT::rolloutPolicy() 方法rollout
*接收到的控制器，而不是仅仅发送回计划状态。
*@param [in] mrtDesiredFrequency：以 Hz 为单位的 MRT 环路频率。这应该始终设置为正数。
*@param [in] mpcDesiredFrequency：以 Hz 为单位的 MPC 环路频率。如果设置为正数，MPC循环
*将被模拟以此频率运行。请注意，这可能不是 MPC 的实时频率。
*/
    MRT_ROS_Dummy_Loop(MRT_ROS_Interface &mrt, scalar_t mrtDesiredFrequency, scalar_t mpcDesiredFrequency = -1);

    /**
     * Destructor.
     */
    virtual ~MRT_ROS_Dummy_Loop() = default;

    /**
*运行虚拟 MRT 循环。
*
*@param [in] initObservation：初始观察。
*@param [in] initTargetTrajectories：初始目标轨迹。
*/
    void run(const SystemObservation &initObservation, const TargetTrajectories &initTargetTrajectories);

    /**
     * Subscribe a set of observers to the dummy loop. Observers are updated in the provided order at the end of each timestep.
     * The previous list of observers is overwritten.
     *
     * @param observers : vector of observers.
     */
    void subscribeObservers(const std::vector<std::shared_ptr<DummyObserver>> &observers) { observers_ = observers; }

  protected:
    /**
*一个用户定义的函数，它在发布之前修改观察。
*
*@param [in] 观察：当前观察。
*/
    virtual void modifyObservation(SystemObservation &observation) {}

  private:
    /**
     *运行一个循环，其中 mpc 优化与系统的正向模拟同步
     */
    void synchronizedDummyLoop(const SystemObservation &initObservation, const TargetTrajectories &initTargetTrajectories);

    /**
     *运行一个循环，其中 mpc 优化和系统模拟是异步的。
     *仿真按照指定的mrtFrequency运行，MPC运行的越快越好。
     */
    void realtimeDummyLoop(const SystemObservation &initObservation, const TargetTrajectories &initTargetTrajectories);

    /** Forward simulates the system from current observation*/
    SystemObservation forwardSimulation(const SystemObservation &currentObservation);

    MRT_ROS_Interface &mrt_;
    std::vector<std::shared_ptr<DummyObserver>> observers_;

    scalar_t mrtDesiredFrequency_;
    scalar_t mpcDesiredFrequency_;
  };

} // namespace ocs2
