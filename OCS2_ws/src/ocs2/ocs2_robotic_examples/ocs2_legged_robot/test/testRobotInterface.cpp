/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-26 21:50:44
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-27 13:09:14
 * @FilePath: /OCS2_Lib_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/test/testLeggedRobotInterface.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include "ocs2_legged_robot/test/AnymalFactoryFunctions.h"

using namespace ocs2;
using namespace legged_robot;
// int add(int a, int b)
// {
//     return a + b;
// }
// TEST(testLeggedRobotInterface, test0)
// {
//     EXPECT_EQ(add(2, 3), 5);
// }

TEST(testLeggedRobotInterface, defaut)
{
    const std::string robotName = "legged_robot";
    std::string taskFile, urdfFile, referenceFile;

    taskFile = "/home/me/OCS2_Lib_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info";
    urdfFile = "/home/me/OCS2_Lib_ws/src/ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf";
    referenceFile = "/home/me/OCS2_Lib_ws/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info";
    LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);
    EXPECT_NEAR(1, 1.0, 1e-6);
}
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    // return 0;
}
