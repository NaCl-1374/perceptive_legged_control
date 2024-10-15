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

#include <Eigen/Core>
#include <array>
#include <cmath>

namespace ocs2
{

  /**
   *计算将 ZYX-Euler 角的导数映射到全局角速度的矩阵
   *
   *@param [in] eulerAngles: ZYX-欧拉角
   *@return 3x3矩阵映射
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(
      const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles)
  {
    const SCALAR_T sz = sin(eulerAngles(0));
    const SCALAR_T cz = cos(eulerAngles(0));
    const SCALAR_T sy = sin(eulerAngles(1));
    const SCALAR_T cy = cos(eulerAngles(1));
    Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
    // clang-format off
  transformationMatrix << SCALAR_T(0.0),            -sz,    cy*cz,
                          SCALAR_T(0.0),             cz,    cy*sz,
                          SCALAR_T(1.0),  SCALAR_T(0.0),      -sy;
    // clang-format on
    return transformationMatrix;
  }

  /**
   *根据 ZYX-Euler 角的导数计算在世界坐标系中表示的角速度
   *
   *@param [in] eulerAngles: ZYX-欧拉角
   *@param [in] derivativesEulerAngles: ZYX-Euler 角度的时间导数
   *@return 以世界坐标系表示的角速度
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getGlobalAngularVelocityFromEulerAnglesZyxDerivatives(
      const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1> &derivativesEulerAngles)
  {
    const SCALAR_T sz = sin(eulerAngles(0));
    const SCALAR_T cz = cos(eulerAngles(0));
    const SCALAR_T sy = sin(eulerAngles(1));
    const SCALAR_T cy = cos(eulerAngles(1));
    const SCALAR_T dz = derivativesEulerAngles(0);
    const SCALAR_T dy = derivativesEulerAngles(1);
    const SCALAR_T dx = derivativesEulerAngles(2);
    return {-sz * dy + cy * cz * dx, cz * dy + cy * sz * dx, dz - sy * dx};
  }

  /**
   *从全局角速度计算 ZYX-Euler 角的导数
   *getGlobalAngularVelocityFromEulerAnglesZyxDerivatives 的反函数
   *
   *此变换对于 y = +-pi /2 是奇异的
   *
   *@param [in] eulerAngles: ZYX-欧拉角
   *@param [in] angularVelocity: 以世界坐标系表示的角速度
   *@return ZYX-欧拉角的导数
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getEulerAnglesZyxDerivativesFromGlobalAngularVelocity(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles,
                                                                                      const Eigen::Matrix<SCALAR_T, 3, 1> &angularVelocity)
  {
    const SCALAR_T sz = sin(eulerAngles(0));
    const SCALAR_T cz = cos(eulerAngles(0));
    const SCALAR_T sy = sin(eulerAngles(1));
    const SCALAR_T cy = cos(eulerAngles(1));
    const SCALAR_T wx = angularVelocity(0);
    const SCALAR_T wy = angularVelocity(1);
    const SCALAR_T wz = angularVelocity(2);
    const SCALAR_T tmp = cz * wx / cy + sz * wy / cy;
    return {sy * tmp + wz, -sz * wx + cz * wy, tmp};//yaw pitch roll
  }

  /**
   *根据 ZYX-Euler 角的导数计算在世界坐标系中表示的角加速度
   *
   *@param [in] eulerAngles: ZYX-欧拉角
   *@param [in] derivativesEulerAngles: ZYX-Euler 角度的时间导数
   *@param [in] secondDerivativesEulerAngles: ZYX-Euler 角度的二阶时间导数
   *@return 以世界坐标系表示的角加速度
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives(
      const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1> &derivativesEulerAngles,
      const Eigen::Matrix<SCALAR_T, 3, 1> &secondDerivativesEulerAngles)
  {
    const SCALAR_T sz = sin(eulerAngles(0));
    const SCALAR_T cz = cos(eulerAngles(0));
    const SCALAR_T sy = sin(eulerAngles(1));
    const SCALAR_T cy = cos(eulerAngles(1));
    const SCALAR_T sx = sin(eulerAngles(2));
    const SCALAR_T cx = cos(eulerAngles(2));
    const SCALAR_T dz = derivativesEulerAngles(0);
    const SCALAR_T dy = derivativesEulerAngles(1);
    const SCALAR_T dx = derivativesEulerAngles(2);
    Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
    Eigen::Matrix<SCALAR_T, 3, 3> derivativeTransformationMatrix;
    // clang-format off
  derivativeTransformationMatrix << SCALAR_T(0), -cz*dz, (-sy*dy)*cz + cy*(-sz*dz),
                                    SCALAR_T(0), -sz*dz,  (-sy*dy)*sz + cy*(cz*dz),
                                    SCALAR_T(0),  SCALAR_T(0),   -cy*dy;
    // clang-format on
    return derivativeTransformationMatrix * derivativesEulerAngles + transformationMatrix * secondDerivativesEulerAngles;
  }

  /**
   *计算将 ZYX-Euler 角的导数映射到局部角速度的矩阵
   *
   *@param [in] eulerAngles: ZYX-欧拉角
   *@return 3x3矩阵映射
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromEulerAnglesZyxDerivativeToLocalAngularVelocity(
      const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles)
  {
    const SCALAR_T sy = sin(eulerAngles(1));
    const SCALAR_T cy = cos(eulerAngles(1));
    const SCALAR_T sx = sin(eulerAngles(2));
    const SCALAR_T cx = cos(eulerAngles(2));
    Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
    // clang-format off
  transformationMatrix <<     -sy,  SCALAR_T(0.0),    SCALAR_T(1.0),
                          cy * sx,             cx,    SCALAR_T(0.0),
                          cx * cy,            -sx,    SCALAR_T(0.0);
    // clang-format on
    return transformationMatrix;
  }

  /**
   *根据 ZYX-Euler 角的导数计算在局部坐标系中表示的角速度
   *
   *@param [in] eulerAngles: ZYX-欧拉角
   *@param [in] derivativesEulerAngles: ZYX-Euler 角度的时间导数
   *@return 以局部坐标系表示的角速度
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getLocalAngularVelocityFromEulerAnglesZyxDerivatives(
      const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1> &derivativesEulerAngles)
  {
    const SCALAR_T sy = sin(eulerAngles(1));
    const SCALAR_T cy = cos(eulerAngles(1));
    const SCALAR_T sx = sin(eulerAngles(2));
    const SCALAR_T cx = cos(eulerAngles(2));
    const SCALAR_T dz = derivativesEulerAngles(0);
    const SCALAR_T dy = derivativesEulerAngles(1);
    const SCALAR_T dx = derivativesEulerAngles(2);
    return {-sy * dz + dx, cy * sx * dz + cx * dy, cx * cy * dz - sx * dy};
  }

  /**
   *根据局部角速度计算 ZYX-Euler 角的导数
   *getLocalAngularVelocityFromEulerAnglesZyxDerivatives 的反函数
   *
   *此变换对于 y = +-pi /2 是奇异的
   *
   *@param [in] eulerAngles: ZYX-欧拉角
   *@param [in] angularVelocity: 以局部帧表示的角速度
   *@return ZYX-欧拉角的导数
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getEulerAnglesZyxDerivativesFromLocalAngularVelocity(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles,
                                                                                     const Eigen::Matrix<SCALAR_T, 3, 1> &angularVelocity)
  {
    const SCALAR_T sy = sin(eulerAngles(1));//y
    const SCALAR_T cy = cos(eulerAngles(1));
    const SCALAR_T sx = sin(eulerAngles(2));//x
    const SCALAR_T cx = cos(eulerAngles(2));
    const SCALAR_T wx = angularVelocity(0);
    const SCALAR_T wy = angularVelocity(1);
    const SCALAR_T wz = angularVelocity(2);
    const SCALAR_T tmp = sx * wy / cy + cx * wz / cy;
    return {tmp, cx * wy - sx * wz, wx + sy * tmp};//yaw pitch roll
  }

} // namespace ocs2