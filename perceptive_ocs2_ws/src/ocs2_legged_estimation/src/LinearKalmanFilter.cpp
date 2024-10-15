/*
 * @Author: NaCl
 * @Date: 2024-04-04 17:18:50
 * @LastEditors: NaCl
 * @LastEditTime: 2024-04-13 20:34:02
 * @FilePath: /perceptive_ocs2_ws/src/ocs2_legged_estimation/src/LinearKalmanFilter.cpp
 * @Description:
 *
 */
//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_legged_estimation/LinearKalmanFilter.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/Rotations.h>

namespace legged
{

  KalmanFilterEstimate::KalmanFilterEstimate(const kinematic_model_t &eeKinematics)
      : StateEstimateBase(eeKinematics),
        numContacts_(NUM_CONTACT_POINTS),             /* 4 */
        dimContacts_(3 * numContacts_),               /* 12 */
        numState_(6 + dimContacts_),                  /* 6+12 pos vel footpos*/
        numObserve_(2 * dimContacts_ + numContacts_), /* 2*12+4 足端位置 速度 高度 */
        tfListener_(tfBuffer_),
        topicUpdated_(false)
  {
    xHat_.setZero(numState_);
    ps_.setZero(dimContacts_);
    vs_.setZero(dimContacts_);
    a_.setIdentity(numState_, numState_);
    b_.setZero(numState_, 3);
    matrix_t c1(3, 6), c2(3, 6);
    c1 << matrix3_t::Identity(), matrix3_t::Zero();
    c2 << matrix3_t::Zero(), matrix3_t::Identity();
    c_.setZero(numObserve_, numState_);
    for (ssize_t i = 0; i < numContacts_; ++i)
    {
      c_.block(3 * i, 0, 3, 6) = c1;
      c_.block(3 * (numContacts_ + i), 0, 3, 6) = c2;
      c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;
    }
    c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

    q_.setIdentity(numState_, numState_);
    p_ = 100. * q_;
    r_.setIdentity(numObserve_, numObserve_);
    feetHeights_.setZero(numContacts_);

    // eeKinematics_->setPinocchioInterface(pinocchioInterface_);

    world2odom_.setRotation(tf2::Quaternion::getIdentity());
    sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);
  }

  vector_t KalmanFilterEstimate::update(const ros::Time &time, const ros::Duration &period)
  {
    scalar_t dt = period.toSec();
    a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();

    b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
    b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();
    q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();
    q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * matrix3_t::Identity();
    q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);

    // const auto& model = pinocchioInterface_.getModel();
    // auto& data = pinocchioInterface_.getData();
    size_t actuatedDofNum = JOINT_COORDINATE_SIZE;

    vector_t qPino(GENCOORDNUM); // euler pos jpos
    vector_t vPino(GENCOORDNUM); // w vel jvel
    // RbdState_ =[euler pos jpos omega vel jvel]
    qPino.setZero();
    qPino.segment<3>(0) = rbdState_.head<3>(); // Only set orientation, let position in origin.
    qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

    vPino.setZero();
    vPino.segment<3>(0) = rbdState_.segment<3>(GENCOORDNUM); // Only set angular velocity, let linear velocity be zero

    // vPino.segment<3>(0) = switched_model::angularVelocitiesToEulerAngleDerivatives<scalar_t>(qPino.segment<3>(0), rbdState_.segment<3>(GENCOORDNUM)); // Only set angular velocity, let linear velocity be zero
    // vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
    //     qPino.segment<3>(3),
    //     rbdState_.segment<3>(GENCOORDNUM)); // Only set angular velocity, let linear velocity be zero

    vPino.tail(actuatedDofNum) = rbdState_.tail(actuatedDofNum);

    // pinocchio::forwardKinematics(model, data, qPino, vPino);
    // pinocchio::updateFramePlacements(model, data);

    const auto eePos = eeKinematics_->feetPositionsInOriginFrame(qPino.head<BASE_COORDINATE_SIZE>(), qPino.tail(JOINT_COORDINATE_SIZE));
    const auto eeVel = eeKinematics_->feetVelocitiesInOriginFrame(qPino.head<BASE_COORDINATE_SIZE>(), vPino.head<BASE_COORDINATE_SIZE>(), qPino.tail(JOINT_COORDINATE_SIZE), vPino.tail(JOINT_COORDINATE_SIZE));

    // const auto eePos = eeKinematics_->getPosition(vector_t());
    // const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

    matrix_t q = matrix_t::Identity(numState_, numState_);
    q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
    q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
    q.block(6, 6, dimContacts_, dimContacts_) = q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;

    matrix_t r = matrix_t::Identity(numObserve_, numObserve_);
    r.block(0, 0, dimContacts_, dimContacts_) = r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;
    r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) =
        r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;
    r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) =
        r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;

    for (int i = 0; i < numContacts_; i++)
    {
      int i1 = 3 * i;

      int qIndex = 6 + i1;
      int rIndex1 = i1;
      int rIndex2 = dimContacts_ + i1;
      int rIndex3 = 2 * dimContacts_ + i;
      bool isContact = contactFlag_[i];

      scalar_t high_suspect_number(100);
      q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
      r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
      r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
      r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

      ps_.segment(3 * i, 3) = -eePos[i];
      ps_.segment(3 * i, 3)[2] += footRadius_;
      vs_.segment(3 * i, 3) = -eeVel[i];
    }
    // ROS_INFO_STREAM("ps_ " << ps_);
    // ROS_INFO_STREAM("vs_ " << vs_);
    // std::cout<<"ps_ " << ps_<<std::endl;
    // std::cout<<"vs_ " << vs_<<std::endl;

    vector3_t g(0, 0, -9.81);
    // switched_model::rotationMatrixBaseToOrigin()
    // vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;
    vector3_t accel = quat_.toRotationMatrix() * linearAccelLocal_ + g;
    vector_t y(numObserve_);
    y << ps_, vs_, feetHeights_;
    xHat_ = a_ * xHat_ + b_ * accel;
    matrix_t at = a_.transpose();
    matrix_t pm = a_ * p_ * at + q;
    matrix_t cT = c_.transpose();
    matrix_t yModel = c_ * xHat_;
    matrix_t ey = y - yModel;
    matrix_t s = c_ * pm * cT + r;

    vector_t sEy = s.lu().solve(ey);
    xHat_ += pm * cT * sEy;

    matrix_t sC = s.lu().solve(c_);
    p_ = (matrix_t::Identity(numState_, numState_) - pm * cT * sC) * pm;

    matrix_t pt = p_.transpose();
    p_ = (p_ + pt) / 2.0;

    //  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
    //    p_.block(0, 2, 2, 16).setZero();
    //    p_.block(2, 0, 16, 2).setZero();
    //    p_.block(0, 0, 2, 2) /= 10.;
    //  }

    if (topicUpdated_)
    {
      updateFromTopic();
      topicUpdated_ = false;
    }
    // std::cout<<"xHat_ " << xHat_<<std::endl;

    updateLinear(xHat_.segment<3>(0), switched_model::rotateVectorOriginToBase(vector3_t(xHat_.segment<3>(3)), quatToXyz(quat_)));

    auto odom = getOdomMsg();
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base";
    publishMsgs(odom);

    return rbdState_;
  }

  void KalmanFilterEstimate::updateFromTopic()
  {
    auto *msg = buffer_.readFromRT();

    tf2::Transform world2sensor;
    world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                             msg->pose.pose.orientation.w));

    if (world2odom_.getRotation() == tf2::Quaternion::getIdentity()) // First received
    {
      tf2::Transform odom2sensor;
      try
      {
        geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("odom", msg->child_frame_id, msg->header.stamp);
        tf2::fromMsg(tf_msg.transform, odom2sensor);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("[LinearKalmanFilter]:%s", ex.what());
        return;
      }
      world2odom_ = world2sensor * odom2sensor.inverse();
      std::cout << "world T odom -> R: \n"
                << world2odom_.getRotation().getW() << " " << world2odom_.getRotation().getX() << " " << world2odom_.getRotation().getY() << " " << world2odom_.getRotation().getZ()
                << " pos :\n"
                << world2odom_.getOrigin().getX() << " " << world2odom_.getOrigin().getY() << " " << world2odom_.getOrigin().getZ() << std::endl;
    }

    // 设置静态变换的参数
    geometry_msgs::TransformStamped static_transform_stamped;
    // 对坐标变换初始化
    static_transform_stamped.header.stamp = ros::Time::now();
    // 父节点
    static_transform_stamped.header.frame_id = "map";
    // 子节点
    static_transform_stamped.child_frame_id = "odom";
    static_transform_stamped.transform.rotation.w = world2odom_.getRotation().getW();
    static_transform_stamped.transform.rotation.x = world2odom_.getRotation().getX();
    static_transform_stamped.transform.rotation.y = world2odom_.getRotation().getY();
    static_transform_stamped.transform.rotation.z = world2odom_.getRotation().getZ();

    static_transform_stamped.transform.translation.x = world2odom_.getOrigin().getX();
    static_transform_stamped.transform.translation.y = world2odom_.getOrigin().getY();
    static_transform_stamped.transform.translation.z = world2odom_.getOrigin().getZ();

    static_broadcaster.sendTransform(static_transform_stamped);


    tf2::Transform base2sensor;
    try
    {
      geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("base", msg->child_frame_id, msg->header.stamp);
      tf2::fromMsg(tf_msg.transform, base2sensor);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();

    tf2::Transform world2base = world2sensor * base2sensor.inverse();

    vector3_t newPos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), odom2base.getOrigin().z());
    // vector3_t newPos(world2base.getOrigin().x(), world2base.getOrigin().y(), world2base.getOrigin().z());

    // const auto& model = pinocchioInterface_.getModel();
    // auto& data = pinocchioInterface_.getData();

    vector_t ocs2Pose(BASE_COORDINATE_SIZE);
    // ocs2Pose.segment<3>(0) = rbdState_.head<3>();
    // ocs2Pose.head<3>(3) = newPos;
    ocs2Pose.head<6>(0) << rbdState_.head<3>(), newPos;
    vector_t ocs2Jpos(JOINT_COORDINATE_SIZE);
    ocs2Jpos = rbdState_.segment(6, JOINT_COORDINATE_SIZE);
    // qPino.tail(JOINT_COORDINATE_SIZE) = rbdState_.segment(6, JOINT_COORDINATE_SIZE);
    // pinocchio::forwardKinematics(model, data, qPino);
    // pinocchio::updateFramePlacements(model, data);
    // const auto eePos=eeKinematics_->feetPositionsInOriginFrame( qPino.head<BASE_COORDINATE_SIZE>(),qPino.tail(actuatedDofNum));
    // const auto eeVel=eeKinematics_->feetVelocitiesInOriginFrame( qPino.head<BASE_COORDINATE_SIZE>(),vPino.head<BASE_COORDINATE_SIZE>(),qPino.tail(actuatedDofNum),vPino.tail(actuatedDofNum));

    xHat_.segment<3>(0) = newPos;
    auto endPos = eeKinematics_->feetPositionsInOriginFrame(ocs2Pose, ocs2Jpos);
    for (size_t i = 0; i < numContacts_; ++i)
    {
      xHat_.segment<3>(6 + i * 3) = endPos[i];
      xHat_(6 + i * 3 + 2) -= footRadius_;
      if (contactFlag_[i])
      {
        feetHeights_[i] = xHat_(6 + i * 3 + 2);
      }
    }

    // auto odom = getOdomMsg();
    // odom.header = msg->header;
    // odom.child_frame_id = "base";
    // publishMsgs(odom);
  }

  void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    buffer_.writeFromNonRT(*msg);
    topicUpdated_ = true;
  }

  nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg()
  {
    nav_msgs::Odometry odom;
    vector3_t zyx = quatToZyx(quat_);
    odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
    odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
    odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
    odom.pose.pose.orientation.x = quat_.x();
    odom.pose.pose.orientation.y = quat_.y();
    odom.pose.pose.orientation.z = quat_.z();
    odom.pose.pose.orientation.w = quat_.w();
    // odom.pose.pose.orientation.x = quat_.x();
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        odom.pose.covariance[i * 6 + j] = p_(i, j);
        odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
      }
    }
    //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
    // vector3_t twist = quat_.toRotationMatrix().transpose() * xHat_.segment<3>(3);
    vector3_t twist = switched_model::rotateVectorOriginToBase(vector3_t(xHat_.segment<3>(3)), quatToXyz(quat_));

    // vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
    // vector_t twist =  xHat_.segment<3>(3);

    odom.twist.twist.linear.x = twist.x();
    odom.twist.twist.linear.y = twist.y();
    odom.twist.twist.linear.z = twist.z();
    odom.twist.twist.angular.x = angularVelLocal_.x();
    odom.twist.twist.angular.y = angularVelLocal_.y();
    odom.twist.twist.angular.z = angularVelLocal_.z();
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
        odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
      }
    }
    return odom;
  }

  void KalmanFilterEstimate::loadSettings(const std::string &taskFile, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "kalmanFilter.";
    if (verbose)
    {
      std::cerr << "\n #### Kalman Filter Noise:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", verbose);
    loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", verbose);
    loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", verbose);
    loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
    loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
    loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
    loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
  }

} // namespace legged
