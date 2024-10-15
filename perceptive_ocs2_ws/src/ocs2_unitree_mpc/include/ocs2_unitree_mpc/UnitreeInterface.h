/*
 * @Author: NaCl
 * @Date: 2024-04-01 12:32:38
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-19 19:47:33
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_unitree_mpc/include/ocs2_unitree_mpc/UnitreeInterface.h
 * @Description:
 *
 */
//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

#include <ocs2_unitree_models/UnitreeModels.h>
#include <ocs2_unitree_models/FrameDeclaration.h>

// add yxy
// #include <ocs2_pinocchio_interface/PinocchioInterface.h>
// #include <ocs2_core/cost/StateCost.h>
// #include <ocs2_legged_self_collishion/ocs2_legged_self_collishion.h>

namespace unitree
{


    std::unique_ptr<switched_model::QuadrupedInterface> getUnitreeInterface(const std::string &urdf, const std::string &taskFolder);

    std::unique_ptr<switched_model::QuadrupedInterface> getUnitreeInterface(const std::string &urdf,
                                                                            switched_model::QuadrupedInterface::Settings settings,
                                                                            const FrameDeclaration &frameDeclaration);

    std::string getConfigFolder(const std::string &configName);

    std::string getTaskFilePath(const std::string &configName);

} // end of namespace unitree
