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

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>
#include <ocs2_core/thread_support/ThreadPool.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/SolverBase.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "ocs2_ddp/DDP_Data.h"
#include "ocs2_ddp/DDP_Settings.h"
#include "ocs2_ddp/riccati_equations/RiccatiModification.h"
#include "ocs2_ddp/search_strategy/SearchStrategyBase.h"

namespace ocs2 {

/**
 * This class is an interface class for the Gauss-Newton DDP based methods.
 */
class GaussNewtonDDP : public SolverBase {
 public:
  /**
*构造器
*@param [in] ddpSettings：包含 Gauss-Newton DDP 算法设置的结构。
*@param [in] rollout: 用于模拟系统动态​​的rollout类。
*@param [in] optimalControlProblem：最优控制问题公式。
*@param [in] initializer：此类为没有控制器可用的时间步初始化状态输入。
*/
  GaussNewtonDDP(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
                 const Initializer& initializer);

  /**
   * Destructor.
   */
  ~GaussNewtonDDP() override;

  void reset() override;

  size_t getNumIterations() const override { return totalNumIterations_; }

  scalar_t getFinalTime() const override { return finalTime_; }

  const OptimalControlProblem& getOptimalControlProblem() const override { return optimalControlProblemStock_.front(); }

  const PerformanceIndex& getPerformanceIndeces() const override { return performanceIndex_; }

  const std::vector<PerformanceIndex>& getIterationsLog() const override { return performanceIndexHistory_; }

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const final;

  const DualSolution* getDualSolution() const override { return &optimizedDualSolution_; }

  const ProblemMetrics& getSolutionMetrics() const override { return optimizedProblemMetrics_; }

  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    return getValueFunctionImpl(time, state, nominalPrimalData_.primalSolution, nominalDualData_.valueFunctionTrajectory);
  }

  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override;

  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    return getStateInputEqualityConstraintLagrangianImpl(time, state, nominalPrimalData_, nominalDualData_);
  }

  MultiplierCollection getIntermediateDualSolution(scalar_t time) const override {
    return getIntermediateDualSolutionAtTime(nominalDualData_.dualSolution, time);
  }

  std::string getBenchmarkingInfo() const override;

  /**
   * Const access to ddp settings
   */
  const ddp::Settings& settings() const { return ddpSettings_; }

 protected:
  /**
*并行多次运行任务的助手（阻塞）
*
*@param [in] taskFunction: 任务函数
*@param [in] N: 运行taskFunction的次数，如果N = 1则在主线程中运行
*/
  void runParallel(std::function<void(void)> taskFunction, size_t N) {
    threadPool_.runParallel([&](int) { taskFunction(); }, N);
  }

  /**
   * Takes the following steps: (1) Computes the Hessian of the Hamiltonian (i.e., Hm) (2) Based on Hm, it calculates
   * the range space and the null space projections of the input-state equality constraints. (3) Based on these two
   * projections, defines the projected LQ model. (4) Finally, defines the Riccati equation modifiers based on the
   * search strategy.
   *
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @param [out] projectedModelData: The projected model data.
   * @param [out] riccatiModification: The Riccati equation modifier.
   */
  void computeProjectionAndRiccatiModification(const ModelData& modelData, const matrix_t& Sm, ModelData& projectedModelData,
                                               riccati_modification::Data& riccatiModification) const;

  /**
*根据搜索策略和算法计算哈密顿量的 Hessian。
*
*@param [in] modelData：模型数据。
*@param [in] Sm: Riccati 矩阵。
*@return 哈密顿量的 Hessian 矩阵。
*/
  virtual matrix_t computeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const = 0;

  /**
   * Calculates an LQ approximate of the optimal control problem for the nodes.
   *
   * @param [in] dualSolution: The dual solution
   * @param [in,out] primalData: The primal Data
   */
  virtual void approximateIntermediateLQ(const DualSolution& dualSolution, PrimalDataContainer& primalData) = 0;

  /**
*使用 primal 和 dual 计算 timeIndex 的控制器，并将结果写回 dstController
*
*@param [in] timeIndex: 当前时间索引
*@param [in] primalData: 用于计算控制器的原始数据
*@param [in] dualData: 用于计算控制器的对偶数据
*@param [out] dstController: 目标控制器
*/
  virtual void calculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                         LinearController& dstController) = 0;

  /**
   * Solves Riccati equations.
   *
   * @param [in] finalValueFunction: The final value of Sm (dfdxx), Sv (dfdx), s (f), for Riccati equation.
   * @return average time step
   */
  virtual scalar_t solveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) = 0;

  /**
*为所有分区求解 Riccati 方程的实现。
*
*@param [in] finalValueFunction Riccati 方程的最终 Sm(dfdxx)、Sv(dfdx)、s(f)。
*@return 平均时间步长
*/
  scalar_t solveSequentialRiccatiEquationsImpl(const ScalarFunctionQuadraticApproximation& finalValueFunction);

  /**
   * Solves a Riccati equations and type_1 constraints error correction compensation for the partition in the given index.
   *
   * @param [in] workerIndex: Current worker index
   * @param [in] partitionInterval: Current active interval
   * @param [in] finalValueFunction The final Sm(dfdxx), Sv(dfdx), s(f), for Riccati equation.
   */
  virtual void riccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                                      const ScalarFunctionQuadraticApproximation& finalValueFunction) = 0;

 private:
  /**
   * Get the State Input Equality Constraint Lagrangian Impl object
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @param [in] primalData: Primal Data
   * @param [in] dualData: DualData
   * @return vector_t
   */
  vector_t getStateInputEqualityConstraintLagrangianImpl(scalar_t time, const vector_t& state, const PrimalDataContainer& primalData,
                                                         const DualDataContainer& dualData) const;

  /**
   * Get the Value Function at time (time) from valueFunctionTrajectory. The the gradient of the value function will be corrected by using
   * the Hessian together with the difference between the current state and the corresponding state stored in the primalSolution.
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @param [in] primalSolution: Primal solution
   * @param [in] valueFunctionTrajectory: A trajectory of the value function quadratic approximation.
   * @return value function
   */
  ScalarFunctionQuadraticApproximation getValueFunctionImpl(
      const scalar_t time, const vector_t& state, const PrimalSolution& primalSolution,
      const std::vector<ScalarFunctionQuadraticApproximation>& valueFunctionTrajectory) const;

  /**
   * @brief Get the Value Function From Cache
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @return ScalarFunctionQuadraticApproximation
   */
  ScalarFunctionQuadraticApproximation getValueFunctionFromCache(scalar_t time, const vector_t& state) const {
    return getValueFunctionImpl(time, state, cachedPrimalData_.primalSolution, cachedDualData_.valueFunctionTrajectory);
  }

  /**
*将系统动力学与 inputPrimalSolution 中的控制器前向积分。一般来说，它使用给定的
*控制策略和初始状态，在时间段[initTime, finalTime]内积分系统动力学。
*但是，如果 inputPrimalSolution 的控制器没有覆盖周期 [initTime, finalTime]，它将使用
*控制器直到控制器的最后时间
*
*@param [in] inputPrimalSolution：它的控制器将用于rollout。
*@param [out] outputPrimalSolution：生成的 PrimalSolution。
*@return True 如果推出成功。
*/
  bool rolloutInitialController(PrimalSolution& inputPrimalSolution, PrimalSolution& outputPrimalSolution);

  /**
*从 inputPrimalSolution 中提取 PrimalSolution 轨迹。一般来说，它会尝试在时间段内提取
*[初始时间，最终时间]。但是，如果 inputPrimalSolution 的 timeTrajectory 没有覆盖周期 [initTime, finalTime]，
*它会提取解决方案，直到时间轨迹的最后一次
*
*@param [in] inputPrimalSolution：它的控制器将用于推出。
*@param [out] outputPrimalSolution：生成的 PrimalSolution。
*@return 如果提取成功则为真。
*/
  bool extractInitialTrajectories(PrimalSolution& inputPrimalSolution, PrimalSolution& outputPrimalSolution);

  /**
*它会检查 primalSolution 的内容，如果它的最终时间小于当前求解器的 finalTime_，
*它将把它与 Initializer 的结果连接起来。
*/
  void rolloutInitializer(PrimalSolution& primalSolution);

  /**
   * Calculates the controller. This method uses the following variables. The method modifies unoptimizedController_.
   */
  void calculateController();

  /**
   * Display rollout info and scores.
   */
  void printRolloutInfo() const;

  /**
   * Calculates the merit function based on the performance index .
   *
   * @param [in] performanceIndex: The performance index which includes the uninitialized merit, cost, and ISEs of constraints.
   * @return The merit function
   */
  scalar_t calculateRolloutMerit(const PerformanceIndex& performanceIndex) const;

  /**
*将非线性问题近似为线性二次问题
*标称状态和控制轨迹。此方法更新以下内容
*变量：
*-线性化系统模型和约束
*-二次成本函数
*-以及约束系数
*-线性化系统模型
*-二次中间成本函数
*-平方化的最终成本
*/
  void approximateOptimalControlProblem();

  /**
   *
   * @param [in] Hm: inv(Hm) defines the oblique projection for state-input equality constraints.
   * @param [in] Dm: The derivative of the state-input constraints w.r.t. input.
   * @param [out] constraintRangeProjector: The projection matrix to the constrained subspace.其为DmDagger ，Hm=Rm inv(Rm)*Dm'*inv(Dm*inv(Rm)*Dm')
   * @param [out] constraintNullProjector: The projection matrix to the null space of constrained. (I-DmDagger*Dm) *inv(Rm) *(I-DmDagger*Dm)^T
   */
  void computeProjections(const matrix_t& Hm, const matrix_t& Dm, matrix_t& constraintRangeProjector,
                          matrix_t& constraintNullProjector) const;

  /** Initialize the constraint penalty coefficients. */
  void initializeConstraintPenalties();

  /**
   * Updates the constraint penalty coefficients.
   * @param [in] equalityConstraintsSSE: SSE of the equality constraints.
   */
  void updateConstraintPenalties(scalar_t equalityConstraintsSSE);

  /** Initializes the nominal primal based on the optimized ones.
   * @return True if the rollout is not purely from the Initializer.
   */
  bool initializePrimalSolution();

  /**
   * Initializes the nominal dual solutions based on the optimized ones and nominal primal solution.
   * Moreover, it updates ProblemMetrics.
   */
  void initializeDualSolutionAndMetrics();

  /** Based on the current LQ solution updates the optimized primal and dual solutions. */
  void takePrimalDualStep(scalar_t lqModelExpectedCost);

  /**
   * Checks convergence of the main loop of DDP.
   *
   * @param [in] isInitalControllerEmpty: Whether the initial controller was empty.
   * @param [in] previousPerformanceIndex: The previous iteration's PerformanceIndex.
   * @param [in] currentPerformanceIndex: The current iteration's PerformanceIndex.
   * @return A pair of (isOptimizationConverged, infoString)
   */
  std::pair<bool, std::string> checkConvergence(bool isInitalControllerEmpty, const PerformanceIndex& previousPerformanceIndex,
                                                const PerformanceIndex& currentPerformanceIndex) const;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) override;

 protected:
  // nominal data
  DualDataContainer nominalDualData_;
  PrimalDataContainer nominalPrimalData_;

  // controller that is calculated directly from dual solution. It is unoptimized because it haven't gone through searching.
  LinearController unoptimizedController_;

  // multi-threading helper variables
  std::atomic_size_t nextTaskId_{0};
  std::atomic_size_t nextTimeIndex_{0};

  scalar_t initTime_ = 0.0;
  scalar_t finalTime_ = 0.0;
  vector_t initState_;

  std::unique_ptr<SearchStrategyBase> searchStrategyPtr_;
  std::vector<OptimalControlProblem> optimalControlProblemStock_;

 private:
  const ddp::Settings ddpSettings_;

  ThreadPool threadPool_;

  unsigned long long int totalNumIterations_{0};

  PerformanceIndex performanceIndex_;
  std::vector<PerformanceIndex> performanceIndexHistory_;

  std::unique_ptr<RolloutBase> initializerRolloutPtr_;
  std::vector<std::unique_ptr<RolloutBase>> dynamicsForwardRolloutPtrStock_;

  // optimized data
  DualSolution optimizedDualSolution_;
  PrimalSolution optimizedPrimalSolution_;
  ProblemMetrics optimizedProblemMetrics_;

  // cached data used for caching the nominal trajectories for which the LQ problem is
  // constructed and solved before terminating run()
  DualDataContainer cachedDualData_;
  PrimalDataContainer cachedPrimalData_;

  struct ConstraintPenaltyCoefficients {
    scalar_t penaltyTol = 1e-3;
    scalar_t penaltyCoeff = 0.0;
  };
  ConstraintPenaltyCoefficients constraintPenaltyCoefficients_;

  // forward pass and backward pass average time step
  scalar_t avgTimeStepFP_ = 0.0;
  scalar_t avgTimeStepBP_ = 0.0;

  // benchmarking
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer backwardPassTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;
  benchmark::RepeatedTimer searchStrategyTimer_;
  benchmark::RepeatedTimer totalDualSolutionTimer_;
};

}  // namespace ocs2
