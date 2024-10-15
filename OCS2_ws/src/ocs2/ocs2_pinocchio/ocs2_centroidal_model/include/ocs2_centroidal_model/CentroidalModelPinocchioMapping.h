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

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "ocs2_centroidal_model/CentroidalModelInfo.h"

namespace ocs2 {

template <typename SCALAR>
class CentroidalModelPinocchioMappingTpl;

using CentroidalModelPinocchioMapping = CentroidalModelPinocchioMappingTpl<scalar_t>;
using CentroidalModelPinocchioMappingCppAd = CentroidalModelPinocchioMappingTpl<ad_scalar_t>;

/**
* Centroidal Dynamics:
*
* State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
* @remark: The linear and angular momenta are expressed with respect to the centroidal frame (a frame centered at
 * the CoM and aligned with the inertial frame).
 * 状态向量 x 包含了线性动量与质量的比值、角动量与质量的比值、基础位置、基础的欧拉姿态（zyx顺序）以及关节位置。这些状态量都是在质心参考系（以质心为中心，与惯性参考系对齐的坐标系）中表达的。
 *
* Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
* @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
*
*
* Pinocchio Joint Positions: qPinocchio = [ base_position, base_orientation_zyx, joint_positions ]'
* @remark: Base position is expressed with respect to the inertial frame
*
* Pinocchio Joint Velocities: vPinocchio = [ base_linear_velocity, base_orientation_zyx_derivatives, joint_velocities ]'
* @remark: Base linear velocity is expressed with respect to the inertial frame
*/
template <typename SCALAR>
class CentroidalModelPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  /**
*构造函数
*@param [in] centroidalModelInfo : 质心模型信息。
*/
  explicit CentroidalModelPinocchioMappingTpl(CentroidalModelInfoTpl<SCALAR> centroidalModelInfo);

  ~CentroidalModelPinocchioMappingTpl() override = default;
  CentroidalModelPinocchioMappingTpl* clone() const override;

  /**设置缓存的pinocchio接口
*@param [in] pinocchioInterface：需要在其上进行计算的 pinocchio 接口。它将为吸气剂保留一个指针。
*@note 必须在调用 getter 之前设置 pinocchio 接口。
*/
  void setPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) override;

  /**
*从机器人状态变量计算匹诺曹函数使用的广义坐标向量 (qPinocchio)
*
*@param [in] state: 系统状态向量
*@return pinocchio 关节位置，这也是机器人具有 ZYX-Euler 角的广义位置
*基本方向的参数化
*/
  vector_t getPinocchioJointPosition(const vector_t& state) const override;

  /**
*从机器人状态和输入变量计算匹诺曹函数使用的广义速度向量 (vPinocchio)
*@param [in] state: 系统状态向量
*@param [in] input: 系统输入向量
*@return pinocchio 关节速度，这也是 pinocchio 关节位置的时间导数
*
*@note 要求 pinocchioInterface 更新为：
*ocs2::updateCentroidalDynamics（interface, info, q, v）
*/
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  /**
*将 pinocchio jacobians dfdq、dfdv 映射到 OCS2 jacobians dfdx、dfdu。
*@param [in] state: 系统状态向量
*@param [in] Jq: jacobian 关于 pinocchio 关节位置
*@param [in] Jv：关于pinocchio 关节速度的雅可比矩阵
*@return 一对 {dfdx, dfdu} 包含关于系统状态和输入的雅可比矩阵
*
*@note 要求 pinocchioInterface 更新为：
*ocs2::updateCentroidalDynamicsDerivatives（interface, info, q, v）
*
*TODO：添加 Jacobian w.r.t 广义加速度作为参数以获得对输入的完全隐式依赖
*/
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  /**
*返回包含质心动力学计算所需的机器人特定信息的结构。
*/
  const CentroidalModelInfoTpl<SCALAR>& getCentroidalModelInfo() const { return centroidalModelInfo_; }

 private:
  CentroidalModelPinocchioMappingTpl(const CentroidalModelPinocchioMappingTpl& rhs);

  const PinocchioInterfaceTpl<SCALAR>* pinocchioInterfacePtr_;
  const CentroidalModelInfoTpl<SCALAR> centroidalModelInfo_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class CentroidalModelPinocchioMappingTpl<scalar_t>;
extern template class CentroidalModelPinocchioMappingTpl<ad_scalar_t>;

}  // namespace ocs2
