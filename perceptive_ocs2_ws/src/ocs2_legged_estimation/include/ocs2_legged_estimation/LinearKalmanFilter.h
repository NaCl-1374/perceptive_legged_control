/*
 * @Author: NaCl
 * @Date: 2024-04-04 17:18:50
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-12 15:49:05
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_estimation/include/ocs2_legged_estimation/LinearKalmanFilter.h
 * @Description: 
 * 
 */
//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "ocs2_legged_estimation/StateEstimateBase.h"

// #include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace legged {
using namespace ocs2;

class KalmanFilterEstimate : public StateEstimateBase {
 public:
  // KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);
  KalmanFilterEstimate(const kinematic_model_t& eeKinematics);

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  void updateFromTopic();

  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  nav_msgs::Odometry getOdomMsg();

  vector_t feetHeights_;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

 private:
  size_t numContacts_, dimContacts_, numState_, numObserve_;

  matrix_t a_, b_, c_, q_, p_, r_;
  vector_t xHat_, ps_, vs_;

  // Topic
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;

};

}  // namespace legged
