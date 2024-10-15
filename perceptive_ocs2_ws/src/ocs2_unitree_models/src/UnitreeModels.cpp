/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-01 12:33:50
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-04-01 12:49:12
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_models/src/UnitreeModels.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by rgrandia on 22.09.20.
//

#include <ocs2_unitree_models/UnitreeModels.h>

#include <unordered_map>

#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_unitree_models/QuadrupedCom.h>
#include <ocs2_unitree_models/QuadrupedInverseKinematics.h>
#include <ocs2_unitree_models/QuadrupedKinematics.h>

#include <ocs2_unitree_models/package_path.h>

namespace unitree {

// std::string toString(UnitreeModel model) {
//   static const std::unordered_map<UnitreeModel, std::string> map{{UnitreeModel::Aliengo, "aliengo"}};
//   return map.at(model);
// }

// UnitreeModel stringToUnitreeModel(const std::string& name) {
//   static const std::unordered_map<std::string, UnitreeModel> map{{"Aliengo", UnitreeModel::Aliengo}};
//   return map.at(name);
// }

// std::string getUrdfPath(UnitreeModel model) {
//   switch (model) {
//     case UnitreeModel::Camel:
//       return getPath() + "/urdf/unitree_camel_rsl.urdf";
//     default:
//       throw std::runtime_error("[UnitreeModels] no default urdf available");
//   }
// }

// std::string getUrdfString(UnitreeModel model) {
//   const auto path = getUrdfPath(model);
//   std::ifstream stream(path.c_str());
//   if (!stream) {
//     throw std::runtime_error("File " + path + " does not exist");
//   }

//   std::string xml_str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
//   return xml_str;
// }

std::unique_ptr<switched_model::InverseKinematicsModelBase> getUnitreeInverseKinematics(const FrameDeclaration& frameDeclaration,
                                                                                       const std::string& urdf) {
  return std::unique_ptr<switched_model::InverseKinematicsModelBase>(
      new QuadrupedInverseKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>> getUnitreeKinematics(const FrameDeclaration& frameDeclaration,
                                                                                         const std::string& urdf) {
  return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::scalar_t>>(
      new QuadrupedKinematics(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>> getUnitreeKinematicsAd(const FrameDeclaration& frameDeclaration,
                                                                                              const std::string& urdf) {
  return std::unique_ptr<switched_model::KinematicsModelBase<ocs2::ad_scalar_t>>(
      new QuadrupedKinematicsAd(frameDeclaration, ocs2::getPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>> getUnitreeComModel(const FrameDeclaration& frameDeclaration,
                                                                                const std::string& urdf) {
  return std::unique_ptr<switched_model::ComModelBase<ocs2::scalar_t>>(
      new QuadrupedCom(frameDeclaration, createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>> getUnitreeComModelAd(const FrameDeclaration& frameDeclaration,
                                                                                     const std::string& urdf) {
  return std::unique_ptr<switched_model::ComModelBase<ocs2::ad_scalar_t>>(
      new QuadrupedComAd(frameDeclaration, createQuadrupedPinocchioInterfaceFromUrdfString(urdf)));
}

}  // namespace unitree
