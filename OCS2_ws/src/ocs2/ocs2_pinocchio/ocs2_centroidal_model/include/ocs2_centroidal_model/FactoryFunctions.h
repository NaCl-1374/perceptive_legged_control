/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-06 13:03:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-20 19:21:14
 * @FilePath: /OCS2_Lib_ws/src/ocs2/ocs2_pinocchio/ocs2_centroidal_model/include/ocs2_centroidal_model/FactoryFunctions.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelInfo.h"

namespace ocs2 {
namespace centroidal_model {

/**
*从 URDF 创建 CentroidalModel PinocchioInterface。
*@param [in] urdfFilePath：机器人 URDF 文件的绝对路径。
*/
PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath);

/**
*从 URDF 创建 CentroidalModel PinocchioInterface。
*@param [in] urdfFilePath：机器人 URDF 文件的绝对路径。
*@param [in] jointNames：任何未在 jointNames 中列出的关节（也称为无关关节）将从 urdf 中删除。
*/
PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath, const std::vector<std::string>& jointNames);

/**
*创建一个标量类型的 CentroidalModelInfo。
*@param [in] interface: 木偶奇遇记界面
*@param [in] type: 模板模型的类型（SRBD 或 FRBD）
*@param [in] nominalJointAngles：SRBD 模型中使用的标称关节角度。
*@param [in] threeDofContactNames：具有 3 个 DoF 触点的末端执行器的名称（力）
*@param [in] sixDofContactNames：具有 6 个 DoF 触点（力 + 扭矩）的末端执行器的名称
*@return CentroidalModelInfo
*/
CentroidalModelInfo createCentroidalModelInfo(const PinocchioInterface& interface, const CentroidalModelType& type,
                                              const vector_t& nominalJointAngles, const std::vector<std::string>& threeDofContactNames,
                                              const std::vector<std::string>& sixDofContactNames);

/**加载 CentroidalModelType从配置文件*/
CentroidalModelType loadCentroidalType(const std::string& configFilePath, const std::string& fieldName = "centroidalModelType");

/**加载配置文件的默认关节状态*/
vector_t loadDefaultJointState(size_t numJointState, const std::string& configFilePath, const std::string& fieldName = "defaultJointState");

}  // namespace centroidal_model
}  // namespace ocs2
