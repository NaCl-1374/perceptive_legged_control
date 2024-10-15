/*
 * @Author: NaCl
 * @Date: 2024-04-04 17:18:50
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-07 23:37:35
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_estimation/src/FromTopicEstimate.cpp
 * @Description: 
 * 
 */

//
// Created by qiayuan on 2022/7/24.
//

#include "ocs2_legged_estimation/FromTopiceEstimate.h"

namespace legged {
FromTopicStateEstimate::FromTopicStateEstimate(const kinematic_model_t& eeKinematics)
    : StateEstimateBase(eeKinematics) {
  ros::NodeHandle nh;
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  updateAngular(quatToXyz(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
                Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
  updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  publishMsgs(odom);

  return rbdState_;
}

}  // namespace legged
