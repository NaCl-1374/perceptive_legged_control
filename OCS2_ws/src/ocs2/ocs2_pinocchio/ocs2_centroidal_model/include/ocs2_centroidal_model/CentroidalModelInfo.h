/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-06 13:03:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-19 21:50:00
 * @FilePath: /OCS2_Lib_ws/src/ocs2/ocs2_pinocchio/ocs2_centroidal_model/include/ocs2_centroidal_model/CentroidalModelInfo.h
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
#include <string>
#include <type_traits>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2
{

  template <typename SCALAR>
  class CentroidalModelInfoTpl;

  using CentroidalModelInfo = CentroidalModelInfoTpl<scalar_t>;
  using CentroidalModelInfoCppAd = CentroidalModelInfoTpl<ad_scalar_t>;

  enum class CentroidalModelType
  {
    FullCentroidalDynamics,
    SingleRigidBodyDynamics
  };

  std::string toString(CentroidalModelType type);
  std::ostream &operator<<(std::ostream &os, CentroidalModelType type);

  template <typename SCALAR>
  struct CentroidalModelInfoTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using scalar_t = SCALAR;
    using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
    using vector3_t = Eigen::Matrix<SCALAR, 3, 1>;
    using matrix3_t = Eigen::Matrix<SCALAR, 3, 3>;

    template <typename T> // Template for conditional compilation using SFINAE
    using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::value, bool>::type;

    CentroidalModelType centroidalModelType;     // 全质心动力学或单刚体动力学 (SRBD)
    size_t numThreeDofContacts;                  // 3DOF 接触，仅力
    size_t numSixDofContacts;                    // 6DOF 触点、力和扭矩
    std::vector<size_t> endEffectorFrameIndices; // 末端执行器框架的索引[3DOF contacts, 6DOF contacts]
    size_t generalizedCoordinatesNum;            // 模型中广义坐标的数量
    size_t actuatedDofNum;                       // 驱动自由度数
    size_t stateDim;                             // 定义system flow map所需的状态数
    size_t inputDim;                             // 定义system flow map所需的输入数
    scalar_t robotMass;                          // 机器人总质量
    vector_t qPinocchioNominal;                  // SRBD 模型中使用的标称机器人配置
    matrix3_t centroidalInertiaNominal;          // SRBD 模型中使用的标称机器人质心惯性（以标称基准坐标系表示）
    vector3_t comToBasePositionNominal;          // 标称 CoM 到 SRBD 模型中使用的基准位置（以标称基准坐标系表示）

    /** Casts CentroidalModelInfo to CentroidalModelInfoCppAD. */
    template <typename T = SCALAR, EnableIfScalar_t<T> = true>
    CentroidalModelInfoCppAd toCppAd() const;
  };

  /* Explicit template instantiation for scalar_t and ad_scalar_t */
  extern template struct CentroidalModelInfoTpl<scalar_t>;
  extern template struct CentroidalModelInfoTpl<ad_scalar_t>;
} // namespace ocs2
