/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-01 12:33:50
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-04-01 12:46:27
 * @FilePath: /perceptive_ocs2_ws/src/unitree_models/include/ocs2_unitree_models/UnitreeModels.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by rgrandia on 22.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/InverseKinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

#include "ocs2_unitree_models/FrameDeclaration.h"

namespace unitree {

enum class UnitreeModel { Aliengo,A1,go1 };

std::string toString(UnitreeModel model);

UnitreeModel stringToUnitreeModel(const std::string& name);

std::string getUrdfPath(UnitreeModel model);
std::string getUrdfString(UnitreeModel model);

std::unique_ptr<switched_model::InverseKinematicsModelBase> getUnitreeInverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getUnitreeKinematics(const FrameDeclaration& frameDeclaration,
                                                                                         const std::string& urdf);

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getUnitreeKinematicsAd(const FrameDeclaration& frameDeclaration,
                                                                                              const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getUnitreeComModel(const FrameDeclaration& frameDeclaration,
                                                                                const std::string& urdf);

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getUnitreeComModelAd(const FrameDeclaration& frameDeclaration,
                                                                                     const std::string& urdf);

}  // namespace unitree
