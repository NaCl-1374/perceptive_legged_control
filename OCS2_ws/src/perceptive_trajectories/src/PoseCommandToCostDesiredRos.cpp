
#include "perceptive_trajectories/PoseCommandToCostDesiredRos.h"
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
namespace switched_model
{

  PoseCommandToCostDesiredRos::PoseCommandToCostDesiredRos(::ros::NodeHandle &nh, const std::string &configFile)
  {

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(configFile, pt);
    targetDisplacementVelocity_ = pt.get<scalar_t>("targetDisplacementVelocity");
    targetRotationVelocity_ = pt.get<scalar_t>("targetRotationVelocity");
    comHeight_ = pt.get<scalar_t>("comHeight");
    ocs2::loadData::loadEigenMatrix(configFile, "defaultJointState", defaultJointState_);
    time_to_target_ = 1.0;

    // 使用 ROS_INFO 打印变量
    ROS_INFO("targetDisplacementVelocity_: %f, targetRotationVelocity_: %f, comHeight_: %f, time_to_target_: %f",
             targetDisplacementVelocity_, targetRotationVelocity_, comHeight_, time_to_target_);

    auto terrainCallback = [this](const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr &msg)
    {
      std::unique_ptr<convex_plane_decomposition::PlanarTerrain> newTerrain(
          new convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg)));

      std::lock_guard<std::mutex> lock(terrainMutex);
      planarTerrainPtr.swap(newTerrain);
    };
    terrainSubscriber_ = nh.subscribe<convex_plane_decomposition_msgs::PlanarTerrain>("/convex_plane_decomposition_ros/planar_terrain", 1, terrainCallback);
  }

  TargetTrajectories PoseCommandToCostDesiredRos::goalToTargetTrajectories(const vector_t &goal, const SystemObservation &observation)
  {
    std::lock_guard<std::mutex> lock(terrainMutex);
    if (planarTerrainPtr)
    {
      const vector_t initSystemState = observation.state;

      // 计算指令

      switched_model::BaseReferenceState initialBaseState{observation.time, switched_model::getPositionInOrigin(switched_model::getBasePose(initSystemState)),
                                                          switched_model::getOrientation(switched_model::getBasePose(initSystemState))};

      auto errYaw = goal(3) - initialBaseState.eulerXyz[2];      // yaw
      auto errX = goal(0) - initialBaseState.positionInWorld[0]; // x
      auto errY = goal(1) - initialBaseState.positionInWorld[1]; // y

      const auto desiredTime = desiredTimeToTarget(errYaw, errX, errY);

      switched_model::WorldReferenceCommand command;
      command.baseHeight = comHeight_;
      command.yawRate = errYaw / desiredTime;
      command.xVelocity = errX / desiredTime;
      command.yVelocity = errY / desiredTime;

      const double dtRef = 0.1;
      const switched_model::BaseReferenceHorizon commandHorizon{dtRef, size_t(desiredTime / dtRef) + 1};

      switched_model::BaseReferenceTrajectory terrainAdaptedBaseReference;

      terrainAdaptedBaseReference = switched_model::generateExtrapolatedBaseReference(commandHorizon, initialBaseState, command, planarTerrainPtr->gridMap,
                                                                                      nominalStanceWidthInHeading_, nominalStanceWidthLateral_);

      ROS_INFO("current Time :%f", observation.time);
      ROS_INFO("Current Yaw X Y :%f  %f  %f", initialBaseState.eulerXyz[2], initialBaseState.positionInWorld[0], initialBaseState.positionInWorld[1]);

      ROS_INFO("Goal    Yaw X Y :%f  %f  %f", goal(3), goal(0), goal(1));
      ROS_INFO("error   Yaw X Y :%f  %f  %f", errYaw, errX, errY);
      ROS_INFO("err/T   Yaw X Y :%f  %f  %f", command.yawRate, command.xVelocity, command.yVelocity);

      ocs2::TargetTrajectories targetTrajectories;
      targetTrajectories.timeTrajectory.push_back(observation.time);

      // targetTrajectories.timeTrajectory.push_back(desiredTime+observation.time);
      // targetTrajectories.stateTrajectory.push_back(initSystemState);
      // targetTrajectories.stateTrajectory.push_back(initSystemState);
      targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
      // targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
      for (int k = 0; k < terrainAdaptedBaseReference.time.size(); ++k)
      {
        targetTrajectories.timeTrajectory.push_back(terrainAdaptedBaseReference.time[k]);

        const auto R_WtoB = switched_model::rotationMatrixOriginToBase(terrainAdaptedBaseReference.eulerXyz[k]);

        Eigen::VectorXd costReference(switched_model::STATE_DIM);
        costReference << terrainAdaptedBaseReference.eulerXyz[k], terrainAdaptedBaseReference.positionInWorld[k],
            R_WtoB * terrainAdaptedBaseReference.angularVelocityInWorld[k], R_WtoB * terrainAdaptedBaseReference.linearVelocityInWorld[k],
            switched_model::getJointPositions(initSystemState);
        targetTrajectories.stateTrajectory.push_back(std::move(costReference));
        targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
      }
      targetTrajectories.timeTrajectory.push_back(observation.time + desiredTime);
      targetTrajectories.stateTrajectory.push_back(targetTrajectories.stateTrajectory.back());
      targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));

      return targetTrajectories;
    }
  }

  TargetTrajectories PoseCommandToCostDesiredRos::cmdVelToTargetTrajectories(const vector_t &cmdVel, const SystemObservation &observation)
  {

    const vector_t initSystemState = observation.state;

    // 计算指令

    switched_model::BaseReferenceState initialBaseState{observation.time, switched_model::getPositionInOrigin(switched_model::getBasePose(initSystemState)),
                                                        switched_model::getOrientation(switched_model::getBasePose(initSystemState))};

    const double dtRef = 0.1* 2;
    const switched_model::BaseReferenceHorizon commandHorizon{dtRef, size_t(time_to_target_ * 2 / dtRef) + 1};

    switched_model::BaseReferenceCommand command;
    command.baseHeight = comHeight_;
    command.yawRate = cmdVel(3);
    command.headingVelocity = cmdVel(0);
    command.lateralVelocity = cmdVel(1);

    switched_model::BaseReferenceTrajectory terrainAdaptedBaseReference;

    std::lock_guard<std::mutex> lock(terrainMutex);
    if (planarTerrainPtr)
    {
      terrainAdaptedBaseReference = switched_model::generateExtrapolatedBaseReference(commandHorizon, initialBaseState, command, planarTerrainPtr->gridMap,
                                                                                      nominalStanceWidthInHeading_, nominalStanceWidthLateral_);
    }
    else
    {
      terrainAdaptedBaseReference = generateExtrapolatedBaseReference(commandHorizon, initialBaseState, command, TerrainPlane());
    }
    ocs2::TargetTrajectories targetTrajectories;
    targetTrajectories.timeTrajectory.push_back(observation.time);
    targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));

    for (int k = 0; k < terrainAdaptedBaseReference.time.size(); ++k)
    {
      targetTrajectories.timeTrajectory.push_back(terrainAdaptedBaseReference.time[k]);

      const auto R_WtoB = switched_model::rotationMatrixOriginToBase(terrainAdaptedBaseReference.eulerXyz[k]);

      Eigen::VectorXd costReference(switched_model::STATE_DIM);
      costReference << terrainAdaptedBaseReference.eulerXyz[k], terrainAdaptedBaseReference.positionInWorld[k],
          R_WtoB * terrainAdaptedBaseReference.angularVelocityInWorld[k], R_WtoB * terrainAdaptedBaseReference.linearVelocityInWorld[k],
          switched_model::getJointPositions(initSystemState);
      targetTrajectories.stateTrajectory.push_back(std::move(costReference));
      targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
    }
    
    targetTrajectories.timeTrajectory.push_back(observation.time + time_to_target_ * 2);
    targetTrajectories.stateTrajectory.push_back(targetTrajectories.stateTrajectory.back());
    targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));

    return targetTrajectories;

    // const vector_t currentPose = observation.state.segment<6>(6);
    // const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
    // vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

    // const scalar_t timeToTarget = TIME_TO_TARGET;
    // const vector_t targetPose = [&]()
    // {
    //   vector_t target(6);
    //   target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    //   target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    //   target(2) = COM_HEIGHT;
    //   target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    //   target(4) = 0;
    //   target(5) = 0;
    //   return target;
    // }();

    // // target reaching duration
    // const scalar_t targetReachingTime = observation.time + timeToTarget;
    // auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
    // trajectories.stateTrajectory[0].head(3) = cmdVelRot;
    // trajectories.stateTrajectory[1].head(3) = cmdVelRot;
    // return trajectories;
  }
  scalar_t PoseCommandToCostDesiredRos::desiredTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const
  {
    scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity_;
    scalar_t displacement = std::sqrt(dx * dx + dy * dy);
    scalar_t displacementTime = displacement / targetDisplacementVelocity_;
    return std::max(rotationTime, displacementTime);
  }
}