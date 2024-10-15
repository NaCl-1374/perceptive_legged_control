/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot/gait/GaitSchedule.h"

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate,
                           scalar_t phaseTransitionStanceTime)
  : modeSchedule_(std::move(initModeSchedule))
  , modeSequenceTemplate_(std::move(initModeSequenceTemplate))
  , phaseTransitionStanceTime_(phaseTransitionStanceTime)
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime,
                                              scalar_t finalTime)
{
  modeSequenceTemplate_ = modeSequenceTemplate;
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();

  // delete the old logic from the index
  if (index < eventTimes.size())
  {
    eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
    modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
  if (!modeSequence.empty() && modeSequence.back() == ModeNumber::STANCE)
  {
    phaseTransitionStanceTime = 0.0;
  }

  if (phaseTransitionStanceTime > 0.0)
  {
    eventTimes.push_back(startTime);
    modeSequence.push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime)
{
  auto& eventTimes = modeSchedule_.eventTimes;  // 获取当前模式调度的事件时间和模式序列
  auto& modeSequence = modeSchedule_.modeSequence;
  // 使用二分查找，找到第一个不小于lowerBoundTime的事件时间的索引
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();
  // 如果找到的索引大于0，说明存在需要删除的旧逻辑
  if (index > 0)
  {
    // delete the old logic from index and set the default start phase to stance
    // 删除索引之前的旧逻辑，并将初始相位设置为STANCE
    // 保留倒数第二个事件时间，确保最后一个相位为STANCE
    eventTimes.erase(eventTimes.begin(),
                     eventTimes.begin() + index - 1);  // keep the one before the last to make it stance
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);

    // set the default initial phase
    // 设置默认的初始相位为STANCE
    modeSequence.front() = ModeNumber::STANCE;
  }

  // Start tiling at time
  // 计算开始平铺的时间，如果事件时间为空，则使用upperBoundTime，否则使用最后一个事件时间
  const auto tilingStartTime = eventTimes.empty() ? upperBoundTime : eventTimes.back();

  // delete the last default stance phase
  // 删除最后一个默认的STANCE相位
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

  // tile the template logic
  // 对模板逻辑进行平铺
  tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime)
{
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const auto& templateTimes = modeSequenceTemplate_.switchingTimes;
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();  // 获取模板中子系统的数量

  // 如果没有定义模板子系统，直接返回，表示最后一个子系统将一直持续
  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0)
  {
    return;
  }
  // 如果已经存在事件时间，并且指定的开始时间不大于最后一个事件时间，抛出运行时错误

  if (!eventTimes.empty() && startTime <= eventTimes.back())
  {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  // 将指定的开始时间添加到事件时间序列中
  eventTimes.push_back(startTime);

  // concatenate from index 在事件时间未达到指定的结束时间之前执行循环
  while (eventTimes.back() < finalTime)
  {
    for (size_t i = 0; i < templateModeSequence.size(); i++)
    {                                                   // 遍历模板模式序列
      modeSequence.push_back(templateModeSequence[i]);  // 将模板模式序列中的模式添加到生成的模式序列中
      scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];  // 计算当前子系统的切换时间差
      eventTimes.push_back(eventTimes.back() + deltaTime);           // 根据切换时间差，更新事件时间序列
    }                                                                // end of i loop
  }                                                                  // end of while loop

  // default final phase
  // 在生成的模式序列中添加一个默认的最后一个相位（STANCE模式），确保最后一个子系统一直持续，直到达到指定的结束时间
  modeSequence.push_back(ModeNumber::STANCE);
}

}  // namespace legged_robot
}  // namespace ocs2
