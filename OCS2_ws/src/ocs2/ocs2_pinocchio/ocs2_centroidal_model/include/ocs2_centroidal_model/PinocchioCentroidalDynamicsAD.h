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

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

namespace ocs2
{

  /**
   *质心动力学：
   *
   *状态：x = [ linear_momentum /mass, angular_momentum /mass, base_position, base_orientation_zyx, joint_positions ]'
   *@remark：线性和角动量是相对于质心框架（以中心为中心的框架）表示的
   *CoM 并与惯性系对齐）。
   *
   *输入：u = [contact_forces, contact_wrenches, joint_velocities]'
   *@remark：接触力和扳手是相对于惯性系表示的。
   */
  class PinocchioCentroidalDynamicsAD final
  {
  public:
    /**
     *构造函数
     *@param [in] pinocchioInterface : pinocchio 接口。
     *@param [in] CentroidalModelInfo : 质心模型信息。
     *@param [in] modelName : 生成模型库的名称
     *@param [in] modelFolder : 保存模型库文件的文件夹
     *@param [in] recompileLibraries : 如果为真，模型库将重新编译。如果为 false，则将加载现有库
     *                                 可用的。
     *@param [in] verbose : 打印信息。
     */
    PinocchioCentroidalDynamicsAD(const PinocchioInterface &pinocchioInterface, const CentroidalModelInfo &info, const std::string &modelName,
                                  const std::string &modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = false);

    /** Copy Constructor */
    PinocchioCentroidalDynamicsAD(const PinocchioCentroidalDynamicsAD &rhs);

    /**
     *计算系统流程图 x_dot = f(x, u)
     *
     *@param time: 时间
     *@param state: 系统状态向量
     *@param input: 系统输入向量
     *@return 系统流程图 x_dot = f(x, u)
     */
    vector_t getValue(scalar_t time, const vector_t &state, const vector_t &input) const;

    /**
     *计算系统流图的一阶近似 x_dot = f(x, u)
     *
     *@param time: 时间
     *@param state: 系统状态向量
     *@param input: 系统输入向量
     *@return 系统流程图的线性近似 x_dot = f(x, u)
     */
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state, const vector_t &input) const;

  private:
    ad_vector_t getValueCppAd(PinocchioInterfaceCppAd &pinocchioInterfaceCppAd, const CentroidalModelPinocchioMappingCppAd &mapping,
                              const ad_vector_t &state, const ad_vector_t &input);

    std::unique_ptr<CppAdInterface> systemFlowMapCppAdInterfacePtr_;
  };

} // namespace ocs2
