/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-07-04 18:32:35
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-02-01 19:59:41
 * @FilePath: /OCS2_Lib_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/test/testLegLogic.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <string>

#include "ocs2_legged_robot/test/AnymalFactoryFunctions.h"
#include <ocs2_robotic_assets/package_path.h>
#include "ocs2_legged_robot/common/ModelSettings.h"
#include "ocs2_legged_robot/package_path.h"

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_legged_robot/gait/LegLogic.h>

namespace
{
    const std::string URDF_FILE = ocs2::robotic_assets::getPath() + "/resources/anymal_c/urdf/anymal.urdf";
    const std::string TASK_FILE = ocs2::legged_robot::getPath() + "/config/mpc/" + "task.info";
    const std::string REFERENCE_FILE = ocs2::legged_robot::getPath() + "/config/command/" + "reference.info";
    const std::string GAIT_FILE = ocs2::legged_robot::getPath() + "/config/command/" + "gait.info";
} // unnamed namespace
using namespace ocs2;
using namespace legged_robot;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

int main(int argc, char **argv)
{
    auto initModeSchedule = loadModeSchedule(REFERENCE_FILE, "initialModeSchedule", false);
    const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(REFERENCE_FILE, "defaultModeSequenceTemplate", false);
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime)
                  { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    //
    const auto standing_trot_ModeSequenceTemplate = loadModeSequenceTemplate(GAIT_FILE, "dynamic_walk", false);
    auto standing_trot_gait = toGait(standing_trot_ModeSequenceTemplate);
    // leglogic test

    // auto ans=extractContactFlags(standing_trot.modeSequence);
    feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg;
    feet_array_t<std::vector<SwingTiming>> swingTimingsPerLeg;

    // Convert mode sequence to a contact flag vector per leg
    const auto contactSequencePerLeg = extractContactFlags(standing_trot_gait.modeSequence);

    // Extract timings per leg
    for (size_t leg = 0; leg < contactTimingsPerLeg.size(); ++leg)
    {
        contactTimingsPerLeg[leg] = extractContactTimings(standing_trot_gait.eventPhases, contactSequencePerLeg[leg]);
        swingTimingsPerLeg[leg] = extractSwingTimings(standing_trot_gait.eventPhases, contactSequencePerLeg[leg]);
    }
    ModeSchedule standing_trot_ModeSchedule(standing_trot_gait.eventPhases, standing_trot_gait.modeSequence);
    std::cout << standing_trot_ModeSchedule << std::endl;


    std::cout << std::fixed << std::setprecision(2);

    // Print table header
    std::cout << std::setw(8) << "Time" << std::setw(20) << "Leg0 LiftOff";

    for (size_t i = 0; i < 4; ++i) {
        std::cout << std::setw(20) << "Leg" + std::to_string(i) + " Phase" << std::setw(15) << "Duration";
    }
    std::cout << '\n';

    // Print table content
    for (int t = 0; t < 100; t++) {
        auto temp = getContactPhasePerLeg(t * 0.01, standing_trot_ModeSchedule);
        auto temp2 = getTimeOfNextLiftOff(t * 0.01, contactTimingsPerLeg[0]);

        std::cout << std::setw(8) << t * 0.01 << std::setw(20) << temp2;

        for (size_t i = 0; i < temp.size(); ++i) {
            std::cout << std::setw(15);
            
            // Set color based on leg index
            switch (i) {
                case 0: std::cout << RED; break;
                case 1: std::cout << GREEN; break;
                case 2: std::cout << YELLOW; break;
                case 3: std::cout << BLUE; break;
                default: std::cout << RESET; break;
            }

            std::cout << "Phase: " << temp[i].phase << " Duration: " << temp[i].duration << RESET;
        }
        std::cout << '\n';
    }

    scalar_t dd = 0.4;
    GaitSchedule Gs(initModeSchedule, defaultModeSequenceTemplate, dd);
    auto temp2 = Gs.getModeSchedule(-10, 10);
    std::cout << temp2 << std::endl;
    auto temp2_2 = Gs.getModeSchedule(0, 10);
    std::cout << temp2_2 << std::endl;
    Gs.insertModeSequenceTemplate(standing_trot_ModeSequenceTemplate, 4., 6.);
    auto temp3 = Gs.getModeSchedule(0, 10);
    std::cout << temp3 << std::endl;
    auto temp2_1 = Gs.getModeSchedule(10.2, 20);
    std::cout << temp2_1 << std::endl;

    // auto ans=extractContactFlags(standing_trot.modeSequence);
    feet_array_t<std::vector<ContactTiming>> contactTimingsPerLeg_;
    feet_array_t<std::vector<SwingTiming>> swingTimingsPerLeg_;

    // Convert mode sequence to a contact flag vector per leg
    const auto contactSequencePerLeg_ = extractContactFlags(temp3.modeSequence);

    // Extract timings per leg
    for (size_t leg = 0; leg < contactTimingsPerLeg_.size(); ++leg)
    {
        contactTimingsPerLeg_[leg] = extractContactTimings(temp3.eventTimes, contactSequencePerLeg_[leg]);
        swingTimingsPerLeg_[leg] = extractSwingTimings(temp3.eventTimes, contactSequencePerLeg_[leg]);
    }

   std::cout << std::fixed << std::setprecision(2);

    // Print table header
    std::cout << std::setw(8) << "Time" << std::setw(20) << "Leg0 LiftOff";

    for (size_t i = 0; i < 4; ++i) {
        std::cout << std::setw(20) << "Leg" + std::to_string(i) + " Phase" << std::setw(15) << "Duration";
    }
    std::cout << '\n';

    // Print table content
    for (int t = 0; t < 100; t++) {
        auto temp = getContactPhasePerLeg(t * 0.01+5, temp3);
        auto temp2 = getTimeOfNextLiftOff(t * 0.01+5, contactTimingsPerLeg[0]);

        std::cout << std::setw(8) << t * 0.01+5 << std::setw(20) << temp2;

        for (size_t i = 0; i < temp.size(); ++i) {
            std::cout << std::setw(15);
            
            // Set color based on leg index
            switch (i) {
                case 0: std::cout << RED; break;
                case 1: std::cout << GREEN; break;
                case 2: std::cout << YELLOW; break;
                case 3: std::cout << BLUE; break;
                default: std::cout << RESET; break;
            }

            std::cout << "Phase: " << temp[i].phase << " Duration: " << temp[i].duration << RESET;
        }
        std::cout << '\n';
    }
    return 0;
}