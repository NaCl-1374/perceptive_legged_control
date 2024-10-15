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

#include <string>
#include <utility>

#include <ocs2_core/Types.h>

namespace ocs2
{

  /** The Kinematics function which maps state-input pair to the end-effector (position, velocity, orientation error) */
  template <typename SCALAR_T>
  class EndEffectorKinematics
  {
  public:
    using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
    using matrix3x_t = Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic>;
    using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
    using quaternion_t = Eigen::Quaternion<SCALAR_T>;

    EndEffectorKinematics() = default;
    virtual ~EndEffectorKinematics() = default;
    virtual EndEffectorKinematics *clone() const = 0;
    EndEffectorKinematics &operator=(const EndEffectorKinematics &) = delete;

    /** Get end-effector IDs (names) */
    virtual const std::vector<std::string> &getIds() const = 0;

    /**
     *获取世界坐标系中末端执行器位置向量
     *
     *@param [in] 状态向量
     *@return 位置向量数组
     */
    virtual std::vector<vector3_t> getPosition(const vector_t &state) const = 0;

    /**
     *获取世界坐标系中末端执行器速度矢量
     *
     *@param [in] state：状态向量
     *@param [in] input：输入向量
     *@return 速度向量数组
     */
    virtual std::vector<vector3_t> getVelocity(const vector_t &state, const vector_t &input) const = 0;

    /**
     *获取世界坐标系中的方向错误
     *
     *@note：要计算误差，请使用 ocs2_robotic_tools/common/RotationTransforms.h 中的 quaternionDistance()
     *
     *@param [in] 状态向量
     *@param [in] referenceOrientation：引用四元数
     *@return 方向错误数组
     */
    virtual std::vector<vector3_t> getOrientationError(const vector_t &state,
                                                       const std::vector<quaternion_t> &referenceOrientations) const = 0;

    /**
     *获取世界坐标系中末端执行器位置的线性近似
     *
     *@param [in] state：状态向量
     *@return 位置函数线性近似数组
     */
    virtual std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t &state) const = 0;

    /**
     *获得世界坐标系中末端执行器速度的线性近似
     *
     *@param [in] state：状态向量
     *@param [in] input：输入向量
     *@return 速度函数线性近似数组
     */
    virtual std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t &state,
                                                                                          const vector_t &input) const = 0;

    /**
     *获取世界坐标系中末端执行器定向误差的线性近似
     *
     *@note：要计算误差和雅可比行列式，请使用 quaternionDistance() 和 quaternionDistanceJacobian()
     *ocs2_robotic_tools/common/RotationTransforms.h
     *
     *@param [in] state：状态向量
     *@param [in] referenceOrientation：引用四元数
     *@return 方向误差线性近似数组
     */
    virtual std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
        const vector_t &state, const std::vector<quaternion_t> &referenceOrientations) const = 0;

  protected:
    EndEffectorKinematics(const EndEffectorKinematics &) = default;
  };

} // namespace ocs2
