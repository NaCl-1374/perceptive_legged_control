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

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_oc/oc_data/DualSolution.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/RolloutBase.h>

#include "ocs2_ddp/DDP_Data.h"

namespace ocs2
{

  /**
   *计算 primalSolution Rollout 中每个点的成本、软约束和约束值。
   *
   *@param [in] 问题：对最优控制问题的引用。
   *@param [in] primalSolution：原始解决方案。
   *@param [in] dualSolution：对偶解决方案的常量参考视图
   *@param [out] problemMetrics：推出的成本、软约束和约束值。
   */
  void computeRolloutMetrics(OptimalControlProblem &problem, const PrimalSolution &primalSolution, DualSolutionConstRef dualSolution,
                             ProblemMetrics &problemMetrics);

  /**
   *计算与给定 ProblemMetrics 关联的 PerformanceIndex。
   *
   *@param [in] timeTrajectory：推出的时间戳。
   *@param [in] problemMetrics：推出的成本、软约束和约束值。
   *
   *@return 轨迹的 PerformanceIndex。
   */
  PerformanceIndex computeRolloutPerformanceIndex(const scalar_array_t &timeTrajectory, const ProblemMetrics &problemMetrics);

  /**
   *正向集成系统动力学与给定的控制器。它使用给定的控制策略和初始状态，
   *在时间段 [initTime, finalTime] 内整合系统动态。
   *
   *@param [in] rollout：对 rollout 类的引用。
   *@param [in] initTime: 初始时间。
   *@param [in] initState: 初始状态。
   *@param [in] finalTime: 最后一次。
   *@param [in, out] primalSolution：生成的原始解决方案。确保 primalSolution::controllerPtr 已设置，因为
   *rollout 是基于存储在 primalSolution 中的控制器执行的。而且，
   *除了 StateTriggeredRollout，还应该设置 primalSolution::modeSchedule。
   *
   *@return 平均时间步长。
   */
  scalar_t rolloutTrajectory(RolloutBase &rollout, scalar_t initTime, const vector_t &initState, scalar_t finalTime,
                             PrimalSolution &primalSolution);

  /**
   *将不受约束的 LQ 系数投影到受约束的 LQ 系数。
   *
   *@param [in] modelData：模型数据。
   *@param [in] constraintRangeProjector：到约束子空间的投影矩阵。
   *@param [in] constraintNullProjector: 到约束零空间的投影矩阵。
   *@param [out] projectedModelData：投影模型数据。
   */
  void projectLQ(const ModelData &modelData, const matrix_t &constraintRangeProjector, const matrix_t &constraintNullProjector,
                 ModelData &projectedModelData);

  /**
   *从给定的原始解决方案中提取范围 [initTime, finalTime] 的原始解决方案。它假设
   *给定范围在输入原解的求解时间内。
   *
   *@note: 控制器字段被忽略。
   *@note: 提取的原始解可以在最终时间有一个事件时间，但在初始时间忽略它。
   *
   *@param [in] timePeriod：应提取解决方案的时间段。
   *@param [in] inputPrimalSolution: 输入PrimalSolution
   *@param [out] outputPrimalSolution：输出PrimalSolution。
   */
  void extractPrimalSolution(const std::pair<scalar_t, scalar_t> &timePeriod, const PrimalSolution &inputPrimalSolution,
                             PrimalSolution &outputPrimalSolution);

  /**
   *计算控制器的最大前馈更新范数。
   *
   *@param [in] controller: 控制策略
   *@return 最大前馈更新范数。
   */
  scalar_t maxControllerUpdateNorm(const LinearController &controller);

  /**
   *计算控制器更新的平方 (IS) 范数的积分。
   *
   *@param [in] 控制器：输入控制器。
   *@return 控制器更新的平方 (IS) 范数的积分。
   */
  scalar_t computeControllerUpdateIS(const LinearController &controller);

  /**
   * Outputs a controller with the same time stamp and gains as unoptimizedController. However, bias is incremented based on:
   * biasArray = unoptimizedController.biasArray + stepLength * unoptimizedController.deltaBiasArray
   */
  void incrementController(scalar_t stepLength, const LinearController &unoptimizedController, LinearController &controller);

  /**
   *从整个时间和事后轨迹中检索当前分区的时间和事后轨迹。
   *生成的时间和事件指标被规范化以从后面开始整合。
   *
   *
   *riccati 方程在时间上向后求解
   *normalizedTimeTrajectory 时间因此以相反的顺序填充负时间，例如：
   *nominalTime = [0.0, 1.0, 2.0, ..., 10.0]
   *normalizedTime = [-10.0, ..., -2.0, -1.0, -0.0]
   *
   *事件索引从当前分区往后数，例如：
   *nominalTime = [0.0, 1.0, 2.0(*), 3.0, 4.0(*)]
   *事件指数 = [2, 4]
   *
   *normalizedTime = [-4.0, -3.0(*), -2.0, -1.0(*), -0.0]
   *normalizedeventIndices = [1, 3]
   *
   *@param [in] partitionInterval: 当前活动间隔
   *@param [in] timeTrajectory: 整个时间轨迹
   *@param [in] postEventIndices: 事件后索引数组
   *@param [out] normalizedTimeTrajectory: 当前区间的归一化时间轨迹
   *@param [out] normalizedPostEventIndices: 当前区间的标准化ost事件索引数组
   */
  void retrieveActiveNormalizedTime(const std::pair<int, int> &partitionInterval, const scalar_array_t &timeTrajectory,
                                    const size_array_t &postEventIndices, scalar_array_t &normalizedTimeTrajectory,
                                    size_array_t &normalizedPostEventIndices);

  /**
   *从时间轨迹中获取分区间隔。间隔定义为 [start, end)。
   *
   *注意，最后一个分区的最右边的索引是 (..., timeArray.size() -1) ，因为最后一个值函数是手动填充的。
   *原因是虽然我们不写入结束索引，但我们必须读取它。将最后一个索引添加到最终分区将
   *导致分段错误。没有简单的方法可以将最终分区与其他分区区分开来，因为根据设计，
   *分区应该被平等对待。
   *
   *每个等于或大于 desiredPartitionPoint 的时间点都应包含在该分区中。这个逻辑在这里是一样的
   *作为事件时间。
   *
   *desiredPartitionPoints最后一次手动填充。不涉及舍入误差。所以使用 == 是安全的
   *浮点数字。使用 std::lower_bound 自然包含最后一个时间点。
   *
   *@param [in] timeTrajectory: 将要划分的时间轨迹
   *@param [in] numWorkers: 工人数，即分区数
   *@return 索引对数组，指示每个分区的开始和结束
   */
  std::vector<std::pair<int, int>> computePartitionIntervals(const scalar_array_t &timeTrajectory, int numWorkers);

  /**
   * Gets a reference to the linear controller from the given primal solution.
   */
  inline LinearController &getLinearController(PrimalSolution &primalSolution)
  {
    assert(dynamic_cast<LinearController *>(primalSolution.controllerPtr_.get()) != nullptr);
    return static_cast<LinearController &>(*primalSolution.controllerPtr_);
  }

  /**
   * Gets a const reference to the linear controller from the given primal solution.
   */
  inline const LinearController &getLinearController(const PrimalSolution &primalSolution)
  {
    assert(dynamic_cast<const LinearController *>(primalSolution.controllerPtr_.get()) != nullptr);
    return static_cast<const LinearController &>(*primalSolution.controllerPtr_);
  }

} // namespace ocs2
