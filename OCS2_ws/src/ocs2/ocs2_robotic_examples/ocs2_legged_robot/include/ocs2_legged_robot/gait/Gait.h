/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-06 13:03:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-01-31 22:10:48
 * @FilePath: /OCS2_Lib_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/include/ocs2_legged_robot/gait/Gait.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

#pragma once

#include <ostream>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2
{
  namespace legged_robot
  {

    /**
     * 步态是由“phases”变量参数化的周期性模式计划.
     *
     * eventPhases仅指示模式的切换，即phase = 0和phase = 1不是eventPhases的一部分。
     * 因此，模式数为phase + 1
     *
     * 时间的转换由持续时间调节
     */
    struct Gait
    {
      /**一个步态周期的时间*/
      scalar_t duration;
      /**沿步态周期 (0.0, 1.0) 中接触模式发生变化的点，大小 N-1*/
      std::vector<scalar_t> eventPhases;
      /**接触方式顺序，尺寸：N*/
      std::vector<size_t> modeSequence;
    };

    /**
     * isValidGait checks the following properties
     * - positive duration
     * - eventPhases are all in (0.0, 1.0)
     * - eventPhases are sorted
     * - the size of the modeSequences is 1 more than the eventPhases.
     */
    bool isValidGait(const Gait &gait);

    /** Check is if the phase is in [0.0, 1.0) */
    bool isValidPhase(scalar_t phase);

    /**将一个阶段包裹到 [0.0, 1.0)*/
    scalar_t wrapPhase(scalar_t phase);

    /** The modes are selected with a closed-open interval: [ ) 获取第一个大于phase的模式索引
    假设 gait.eventPhases 是一个有序的向量： [0.1, 0.3, 0.5, 0.7, 0.9]，并且 phase 的值为 0.6。

    std::upper_bound(gait.eventPhases.begin(), gait.eventPhases.end(), phase) 将找到第一个大于 0.6 的元素，即 0.7。
    firstLargerValueIterator 将成为指向 0.7 的迭代器。
    firstLargerValueIterator - gait.eventPhases.begin() 将返回 3，因为 0.7 在序列中的索引为 3。
    整体返回值将是 3（转换为整数）。如果超出最大则返回最大索引+1
    */
    int getModeIndexFromPhase(scalar_t phase, const Gait &gait);

    /**从phase变量获取活动模式*/
    size_t getModeFromPhase(scalar_t phase, const Gait &gait);

    /** Returns the time left in the gait based on the phase variable 步态周期的剩余时间*/
    scalar_t timeLeftInGait(scalar_t phase, const Gait &gait);

    /** Returns the time left in the current based on the phase variable 返回该phase时的对应模式所剩余的时间*/
    scalar_t timeLeftInMode(scalar_t phase, const Gait &gait);

    /** Print gait */
    std::ostream &operator<<(std::ostream &stream, const Gait &gait);

  } // namespace legged_robot
} // namespace ocs2
