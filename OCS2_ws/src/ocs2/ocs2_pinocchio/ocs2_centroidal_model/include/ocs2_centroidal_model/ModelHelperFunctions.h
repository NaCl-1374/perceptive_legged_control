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

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelInfo.h"

namespace ocs2
{

  /**
   *获取与浮动基变量对应的质心动量矩阵子块的逆。
   *Ab_inv = [ 1/m I_{3,3}, -1/m*Ab_12*Ab_22^(-1),
   *O_{3,3}, Ab_22^(-1) ]
   *
   *@param [in] A(q): 质心动量矩阵
   *@return Ab_inv(q): A(q) 的 6x6 左块的逆
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6> &Ab);

  /**
   *为 FullCentroidalDynamics 模型和 SingleRigidBodyDynamics 模型更新 data.Ag 中的质心动量矩阵和 data.com[0] 中的 CoM 位置
   *@param [in] interface: 包含模型+数据的pinocchio机器人界面
   *@param [in] info: 质心模型信息
   *@param [in] q: pinocchio关节位置（广义坐标）
   *
   *@remark: 这个函数也在内部调用：
   *pinocchio::forwardKinematics（模型，数据，q）
   *pinocchio::computeJointJacobians(model, data, q) （仅适用于 FullCentroidalDynamics 案例）
   *pinocchio::updateFramePlacements（模型，数据）
   */
  template <typename SCALAR_T>
  void updateCentroidalDynamics(PinocchioInterfaceTpl<SCALAR_T> &interface, const CentroidalModelInfoTpl<SCALAR_T> &info,
                                const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> &q);

  /**
   *更新 FullCentroidalDynamics 模型和 SingleRigidBodyDynamics 模型的质心动量导数（例如在 data.dHdq 中）
   *@param [in] interface: 包含模型+数据的pinocchio机器人界面
   *@param [in] info: 质心模型信息
   *@param [in] q: pinocchio关节位置（广义坐标）
   *@param [in] v: pinocchio关节速度（广义坐标的导数）
   *
   *@remark: 这个函数也在内部调用：
   *pinocchio::正向运动学（模型、数据、q）
   *pinocchio：：computeJointJacobians（模型，数据，q）
   *pinocchio::updateFramePlacements（模型，数据）
   */
  template <typename SCALAR_T>
  void updateCentroidalDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T> &interface, const CentroidalModelInfoTpl<SCALAR_T> &info,
                                           const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> &q,
                                           const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> &v);

  /**
   *计算映射的导数（ZYX-欧拉角导数 --> 全局角速度）相对于base方向（ZYX-欧拉角）
   *
   *@param [in] eulerAngles：从 qPinocchio 中提取的 ZYX-Euler 角
   *@return 表示映射的导数 w.r.t ZYX-Euler 角的张量
   */
  template <typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles);

  /**
   *计算旋转矩阵（基础坐标系 --> 世界坐标系）相对于base方向（ZYX-Euler 角）的导数
   *
   *@param [in] eulerAngles：从 qPinocchio 中提取的 ZYX-Euler 角
   *@return 代表旋转矩阵的导数的张量 w.r.t ZYX-Euler 角度
   */
  template <typename SCALAR_T>
  std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> getRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1> &eulerAngles);

  /**
   *计算质心动量相对于基本方向的导数（在 ZYX-欧拉角中）
   *
   *@param [in] interface: 包含模型+数据的pinocchio机器人界面
   *@param [in] info: 质心模型信息
   *@param [in] q: pinocchio关节位置（广义坐标）
   *@param [in] v: pinocchio关节速度（广义坐标的导数）
   *@return 质心动量的导数 w.r.t ZYX-Euler 角
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 6, 3> getCentroidalMomentumZyxGradient(const PinocchioInterfaceTpl<SCALAR_T> &interface,
                                                                 const CentroidalModelInfoTpl<SCALAR_T> &info,
                                                                 const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> &q,
                                                                 const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> &v);

  /**
   *从 pinocchioInterface 数据返回质心动量矩阵
   *@param [in] interface: 包含模型+数据的pinocchio机器人界面
   *@return data.Ag 的质心动量矩阵
   *
   *@note 要求 pinocchioInterface 更新为：
   *ocs2::updateCentroidalDynamics（接口、信息、q）
   */
  template <typename SCALAR_T>
  const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> &getCentroidalMomentumMatrix(const PinocchioInterfaceTpl<SCALAR_T> &interface);

  /**
   *计算 CoM 到接触点位置在世界系的
   *
   *@param [in] interface: 包含模型+数据的pinocchio机器人界面
   *@param [in] info: 质心模型信息
   *@param [in] contactIndex: 联系点索引
   *@return: 接触点的位置 w.r.t CoM 在世界框架中表示
   *
   *@note 要求 pinocchioInterface 更新为：
   *ocs2::updateCentroidalDynamics（接口、信息、q）
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> getPositionComToContactPointInWorldFrame(const PinocchioInterfaceTpl<SCALAR_T> &interface,
                                                                         const CentroidalModelInfoTpl<SCALAR_T> &info, size_t contactIndex);

  /**
   *计算 CoM 到世界框架中的接触点平移雅可比行列式
   *
   *@param [in] interface: 包含模型+数据的pinocchio机器人界面
   *@param [in] info: 质心模型信息
   *@param [in] contactIndex: 联系点索引
   *@return: CoM to contact point translational Jacobian expressed in world frame
   *
   *@note 要求 pinocchioInterface 更新为：
   *ocs2::updateCentroidalDynamics(interface, info, q)（应首先调用）
   *pinocchio：：computeJointJacobians（模型，数据，q）
   *pinocchio::updateFramePlacements（模型，数据）
   */
  // TODO：需要在此处复制数据，因为 getFrameJacobian() 会修改数据。将在 pinocchio 版本 3 中修复。
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> getTranslationalJacobianComToContactPointInWorldFrame(
      const PinocchioInterfaceTpl<SCALAR_T> &interface, const CentroidalModelInfoTpl<SCALAR_T> &info, size_t contactIndex);

  /**
   *计算在质心坐标系中表示的normalized质心动量（线性+角度）的导数
   *
   *@param [in] interface: 包含模型+数据的pinocchio机器人界面
   *@param [in] info: 质心模型信息
   *@param [in] input: 系统输入向量
   *@return: normalized质心动量的时间导数
   *
   *@note 要求 pinocchioInterface 更新为：
   *ocs2::updateCentroidalDynamics（接口、信息、q）
   */
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 6, 1> getNormalizedCentroidalMomentumRate(const PinocchioInterfaceTpl<SCALAR_T> &interface,
                                                                    const CentroidalModelInfoTpl<SCALAR_T> &info,
                                                                    const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> &input);

} // namespace ocs2

#include "implementation/ModelHelperFunctionsImpl.h"
