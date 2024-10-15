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
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

namespace ocs2
{

  /**
   *质心动力学：
   *
   *状态：x = [ linear_momentum /mass, angular_momentum /mass, base_position, base_orientation_zyx, joint_positions ]'
   *@remark：线性和角动量是相对于质心框架（以中心为中心的框架）表示的
   *CoM 并与惯性系对齐）。normalized化的动量
   *
   *输入：u = [contact_forces, contact_wrenches, joint_velocities]'
   *@remark：接触力和扳手是相对于惯性系表示的。
   */
  class PinocchioCentroidalDynamics final
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
    using Matrix3x = Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>;
    using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
    using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

    /**
     *构造函数
     *@param [in] CentroidalModelInfo : 质心模型信息。
     */
    explicit PinocchioCentroidalDynamics(CentroidalModelInfo info);

    /** Copy Constructor */
    PinocchioCentroidalDynamics(const PinocchioCentroidalDynamics &rhs);

    /**设置用于缓存的pinocchio接口。
     *@param [in] pinocchioInterface：需要在其上进行计算的 pinocchio 接口。它将为getters保留一个指针。
     *@note 必须在调用 getter 之前设置 pinocchio 接口。
     */
    void setPinocchioInterface(const PinocchioInterface &pinocchioInterface)
    {
      pinocchioInterfacePtr_ = &pinocchioInterface;
      mapping_.setPinocchioInterface(pinocchioInterface);
    }

    /**
     *计算system flow map x_dot = f(x, u)
     *
     *@param time: 时间
     *@param state: 系统状态向量
     *@param input: 系统输入向量
     *@return system flow map x_dot = f(x, u)
     *
     *@note 要求 pinocchioInterface 更新为：
     *ocs2::updateCentroidalDynamics（接口、信息、q）
     */
    vector_t getValue(scalar_t time, const vector_t &state, const vector_t &input);

    /**
     * Computes first order approximation of the system flow map x_dot = f(x, u)
     *
     * @param time: time
     * @param state: system state vector
     * @param input: system input vector
     * @return linear approximation of system flow map x_dot = f(x, u)
     *
     * @note requires pinocchioInterface to be updated with:
     *       ocs2::updateCentroidalDynamicsDerivatives(interface, info, q, v)
     */
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state, const vector_t &input);

  private:
    /**
     *计算以质心坐标系表示的normalized质心动量率（线性+角度）的梯度 //normalized即动量/质量 h/mass
     *
     *@param [in] state: 系统状态向量
     *@param [in] input: 系统输入向量
     *@return：normalized质心动量的时间导数（线性近似所需）
     */
    void computeNormalizedCentroidalMomentumRateGradients(const vector_t &state, const vector_t &input);

    const PinocchioInterface *pinocchioInterfacePtr_;
    CentroidalModelPinocchioMapping mapping_;

    // 系统动力学的偏导数
    Matrix3x normalizedLinearMomentumRateDerivativeQ_;
    Matrix3x normalizedAngularMomentumRateDerivativeQ_;
    Matrix3x normalizedLinearMomentumRateDerivativeInput_;
    Matrix3x normalizedAngularMomentumRateDerivativeInput_;
  };
} // namespace ocs2
