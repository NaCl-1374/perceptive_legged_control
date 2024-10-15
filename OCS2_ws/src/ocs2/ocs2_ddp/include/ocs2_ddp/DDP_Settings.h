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

#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/Integrator.h>

#include "ocs2_ddp/search_strategy/StrategySettings.h"

namespace ocs2 {
namespace ddp {

/**
 * @brief The DDP algorithm enum
 * Enum used in selecting either SLQ, ILQR algorithms.
 */
enum class Algorithm { SLQ, ILQR };

/**
 * Get string name of DDP algorithm type
 * @param [in] type: DDP algorithm type enum
 */
std::string toAlgorithmName(Algorithm type);

/**
 * Get DDP algorithm type from string name, useful for reading config file
 * @param [in] name: DDP algorithm name
 */
Algorithm fromAlgorithmName(std::string name);

/**
 * This structure contains the settings for the DDP algorithm.
 */
struct Settings {
  /** It should be either SLQ or ILQR */
  Algorithm algorithm_ = Algorithm::SLQ;

  /** Number of threads used in the multi-threading scheme. 多线程方案中使用的线程数。*/
  size_t nThreads_ = 1;
  /** Priority of threads used in the multi-threading scheme. 多线程方案中使用的线程的优先级*/
  int threadPriority_ = 99;

  /** Maximum number of iterations of DDP. d d p的最大迭代次数*/
  size_t maxNumIterations_ = 15;
  /** This value determines the termination condition based on the minimum relative changes of the cost. 该值根据成本的最小相对变化确定终止条件*/
  scalar_t minRelCost_ = 1e-3;
  /** This value determines the tolerance of constraint's ISE (Integral of Square Error). 这个值决定了约束的容忍度(误差平方的积分)。*/
  scalar_t constraintTolerance_ = 1e-3;

  /** This value determines to display the log output DDP. 该值决定是否显示日志输出d d p*/
  bool displayInfo_ = false;
  /** This value determines to display the a summary log of DDP. 此值决定显示d d p的摘要日志*/
  bool displayShortSummary_ = false;
  /** Check the numerical stability of the algorithms for debugging purpose. 检查算法的数值稳定性，以便调试*/
  bool checkNumericalStability_ = true;
  /** Printing rollout trajectory for debugging. 打印滚动轨迹用于调试*/
  bool debugPrintRollout_ = false;

  /** This value determines the absolute tolerance error for ode solvers. 这个值决定了代码求解器的绝对容差*/
  scalar_t absTolODE_ = 1e-9;
  /** This value determines the relative tolerance error for ode solvers.  该值决定了代码求解器的相对容差*/
  scalar_t relTolODE_ = 1e-6;
  /** This value determines the maximum number of integration points per a second for ode solvers. 该值决定了代码求解器每秒的最大积分点数*/
  size_t maxNumStepsPerSecond_ = 10000;
  /** The integration time step for Riccati equation which is used for fixed timestep integration scheme. riccati方程的积分时间步长用于固定时间步长的积分方案*/
  scalar_t timeStep_ = 1e-2;
  /** The backward pass integrator type: SLQ uses it for solving Riccati equation and ILQR uses it for discretizing LQ approximation. 后向传递积分器类型:s1q用它来求解riccati方程i1q用它来离散lq近似。*/
  IntegratorType backwardPassIntegratorType_ = IntegratorType::ODE45;

  /** The initial coefficient of the quadratic penalty function in the merit function. It should be greater than one. 二次罚项的初始系数*/
  scalar_t constraintPenaltyInitialValue_ = 2.0;
  /** The rate that the coefficient of the quadratic penalty function in the merit function grows. It should be greater than one. 二次罚项的系数增长率*/
  scalar_t constraintPenaltyIncreaseRate_ = 2.0;

  /** If true, terms of the Riccati equation will be pre-computed before interpolation in the flow-map 如果为真，riccati方程的项将在flow-map插值之前预先计算*/
  bool preComputeRiccatiTerms_ = true;

  /** Use either the optimized control policy (true) or the o
   * 
   * ptimized state-input trajectory (false). 使用优化的控制策略(true)或优化的状态输入轨迹(false)*/
  bool useFeedbackPolicy_ = false;

  /** The risk sensitivity coefficient for risk aware DDP. */
  scalar_t riskSensitiveCoeff_ = 0.0;

  /** Determines the strategy for solving the subproblem. There are two choices line-search strategy and levenberg_marquardt strategy. 决定解决子问题的策略。有两种选择:直线搜索策略和levenberg_marquardt策略*/
  search_strategy::Type strategy_ = search_strategy::Type::LINE_SEARCH;
  /** The line-search strategy settings. */
  line_search::Settings lineSearch_;
  /** The levenberg_marquardt strategy settings. */
  levenberg_marquardt::Settings levenbergMarquardt_;

};  // end of DDP_Settings

/**
 * This function loads the "DDP_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
 *
 * It has the following format: <br>
 * slq  <br>
 * {  <br>
 *   maxIteration        value    <br>
 *   minLearningRate     value    <br>
 *   maxLearningRate     value    <br>
 *   minRelCost          value    <br>
 *   (and so on for the other fields) <br>
 * }  <br>
 *
 * If a value for a specific field is not defined it will set to the default value defined in "DDP_Settings".
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
 * @return The DDP settings.
 */
Settings loadSettings(const std::string& filename, const std::string& fieldName = "ddp", bool verbose = true);

}  // namespace ddp
}  // namespace ocs2
