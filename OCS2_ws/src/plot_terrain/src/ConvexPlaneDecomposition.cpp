#include <ros/init.h>
#include <ros/node_handle.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <convex_plane_decomposition/LoadGridmapFromImage.h>
#include <memory>

#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include "convex_plane_decomposition_ros/MessageConversion.h"
#include <convex_plane_decomposition_ros/ParameterLoading.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <convex_plane_decomposition/Timer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/smoothing.hpp>
#include <convex_plane_decomposition/Timer.h>

// 全局变量用于存储位置数据
grid_map::Position position(0, 0);
// 回调函数，当接收到消息时被调用
// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//     // 提取所有数据
//     double x = msg->pose.pose.position.x;
//     double y = msg->pose.pose.position.y;
//     position << x, y;
// }

// Noise
double noiseUniform;
double noiseGauss;
double outlierPercentage;
bool blur;
grid_map::GridMap::Matrix noiseLayer;

void createNoise(size_t row, size_t col)
{
  // Box-Muller Transform
  grid_map::GridMap::Matrix u1 = 0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix u2 = 0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix gauss01 =
      u1.binaryExpr(u2, [&](float v1, float v2)
                    { return std::sqrt(-2.0f * log(v1)) * cos(2.0f * M_PIf32 * v2); });

  noiseLayer = noiseUniform * grid_map::GridMap::Matrix::Random(row, col) + noiseGauss * gauss01;
}
// Parameters
std::string elevationMapTopic_;
std::string elevationLayer_;
std::string targetFrameId_;
double subMapWidth_;
double subMapLength_;
bool publishToController_;
double resolution_;
std::unique_ptr<convex_plane_decomposition::PlaneDecompositionPipeline> planeDecompositionPipeline_;
tf2_ros::Buffer tfBuffer;

bool loadParameters(const ros::NodeHandle &nodeHandle)
{
  if (!nodeHandle.getParam("elevation_topic", elevationMapTopic_))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `elevation_topic`.");
    return false;
  }
  if (!nodeHandle.getParam("target_frame_id", targetFrameId_))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `target_frame_id`.");
    return false;
  }
  if (!nodeHandle.getParam("height_layer", elevationLayer_))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `height_layer`.");
    return false;
  }
  if (!nodeHandle.getParam("submap/width", subMapWidth_))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `submap/width`.");
    return false;
  }
  if (!nodeHandle.getParam("submap/length", subMapLength_))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `submap/length`.");
    return false;
  }
  if (!nodeHandle.getParam("publish_to_controller", publishToController_))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `publish_to_controller`.");
    return false;
  }
  if (!nodeHandle.getParam("noiseGauss", noiseGauss))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `noiseGauss`.");
    return 1;
  }
  if (!nodeHandle.getParam("noiseUniform", noiseUniform))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `noiseUniform`.");
    return 1;
  }
  if (!nodeHandle.getParam("blur", blur))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `blur`.");
    return 1;
  }
  if (!nodeHandle.getParam("outlier_percentage", outlierPercentage))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `outlier_percentage`.");
    return 1;
  }
  if (!nodeHandle.getParam("resolution", resolution_))
  {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `outlier_percentage`.");
    return 1;
  }

  convex_plane_decomposition::PlaneDecompositionPipeline::Config config;
  config.preprocessingParameters = convex_plane_decomposition::loadPreprocessingParameters(nodeHandle, "preprocessing/");
  config.contourExtractionParameters = convex_plane_decomposition::loadContourExtractionParameters(nodeHandle, "contour_extraction/");
  config.ransacPlaneExtractorParameters = convex_plane_decomposition::loadRansacPlaneExtractorParameters(nodeHandle, "ransac_plane_refinement/");
  config.slidingWindowPlaneExtractorParameters = convex_plane_decomposition::loadSlidingWindowPlaneExtractorParameters(nodeHandle, "sliding_window_plane_extractor/");
  config.postprocessingParameters = convex_plane_decomposition::loadPostprocessingParameters(nodeHandle, "postprocessing/");

  planeDecompositionPipeline_ = std::make_unique<convex_plane_decomposition::PlaneDecompositionPipeline>(config);
  // resolution_ = config.preprocessingParameters.resolution;
  return true;
}

// ROS communication
ros::Subscriber elevationMapSubscriber_;
ros::Publisher filteredmapPublisher_;
ros::Publisher boundaryPublisher_;
ros::Publisher insetPublisher_;
ros::Publisher regionPublisher_;
ros::Publisher elevationmapPublisher_;
int main(int argc, char *argv[])
{
  const std::string path(__FILE__);
  const std::string ocs2_anymal = path.substr(0, path.find_last_of("/")) + "/../../";
  const std::string terrainFolder = ocs2_anymal + "plot_terrain/data/";

  const std::string robotName = "anymal";

  ros::init(argc, argv, "ConvexPlaneDecomposition_test");
  ros::NodeHandle nodeHandle("~");
  std::cout << nodeHandle.getNamespace() << std::endl;

  std::string pubName{"planar_terrain"};
  nodeHandle.getParam("pubTopicName", pubName);

  ROS_INFO("publish Topic Name is  %s", pubName.c_str());

  ROS_INFO("elevationMapTopic_ Topic Name is  %s", elevationMapTopic_.c_str());

  // ros::Subscriber sub = nodeHandle.subscribe("/tracking_camera/odom/sample", 10, odomCallback);

  tf2_ros::TransformListener tfListener(tfBuffer);

  bool parametersLoaded = loadParameters(nodeHandle);
  if (parametersLoaded)
  {
    filteredmapPublisher_ = nodeHandle.advertise<grid_map_msgs::GridMap>("filtered_map", 1);
    boundaryPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("boundaries", 1);
    insetPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("insets", 1);
    regionPublisher_ = nodeHandle.advertise<convex_plane_decomposition_msgs::PlanarTerrain>(pubName, 1);
    elevationmapPublisher_ = nodeHandle.advertise<grid_map_msgs::GridMap>(elevationMapTopic_, 1);
  }

  // 载入地图
  const std::string frameId{"odom"};
  bool debug = false;
  std::string terrainFile{""};
  nodeHandle.getParam("/terrain_name", terrainFile);
  nodeHandle.getParam("debug", debug);

  double heightScale{1.0};
  nodeHandle.getParam("/terrain_scale", heightScale);
  auto gridMap = convex_plane_decomposition::loadGridmapFromImage(terrainFolder + "/" + terrainFile, elevationLayer_, frameId,
                                                                  resolution_, heightScale);
  gridMap.get(elevationLayer_).array() -= gridMap.atPosition(elevationLayer_, {0., 0.});

  if (blur)
  {
    // Copy!
    auto originalMap = gridMap.get(elevationLayer_);

    // Blur (remove nan -> filter -> put back nan
    grid_map::inpainting::minValues(gridMap, elevationLayer_, "i");
    grid_map::smoothing::boxBlur(gridMap, "i", elevationLayer_, 3, 1);
    gridMap.get(elevationLayer_) = (originalMap.array().isFinite()).select(gridMap.get(elevationLayer_), originalMap);
  }

  auto &elevation = gridMap.get(elevationLayer_);
  if (noiseLayer.size() != elevation.size())
  {
    createNoise(elevation.rows(), elevation.cols());
  }

  elevation += noiseLayer;

  gridMap[elevationLayer_] = elevation;

  ROS_INFO("From Image import Map Done");
  ROS_INFO("gridMap it has this layers");
  std::vector<std::string> layers;
  layers = gridMap.getLayers();
  for (const auto &layer : layers)
  {
    ROS_INFO("- %s", layer.c_str());
  }
  double frequency;

  if (!nodeHandle.getParam("frequency", frequency))
  {
    ROS_ERROR("[ConvexPlaneDecompositionNode] Could not read parameter `frequency`.");
    return 1;
  }
  bool isprint = false;
  ros::Rate rate(frequency);
  int iter = 0;
  while (ros::ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      // Lookup the transform from "odom" to "base_link"
      std::string errorMsg;
      grid_map_msgs::GridMap RawoutputMessage;
      grid_map::GridMapRosConverter::toMessage(gridMap, RawoutputMessage);
      elevationmapPublisher_.publish(RawoutputMessage);
      ros::Time timeStamp = ros::Time(0); // Use Time(0) to get the latest transform.
      if (tfBuffer.canTransform("world", "base", timeStamp, &errorMsg))
      {

        transformStamped = tfBuffer.lookupTransform("world", "base", ros::Time(0));
        grid_map::Position submapPosition(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
        if (debug)
          ROS_INFO("Translation: [%.2f, %.2f]",
                   transformStamped.transform.translation.x,
                   transformStamped.transform.translation.y);
        bool success;
        grid_map::GridMap elevationMap = gridMap.getSubmap(submapPosition, Eigen::Array2d(subMapLength_, subMapWidth_), success);
        if (debug)
          ROS_INFO("getSubmap it has this layers");

        const grid_map::Matrix elevationRaw = elevationMap.get(elevationLayer_);
        // Run pipeline.
        planeDecompositionPipeline_->update(std::move(elevationMap), elevationLayer_);
        auto &planarTerrain = planeDecompositionPipeline_->getPlanarTerrain();
        if (publishToController_)
        {
          regionPublisher_.publish(toMessage(planarTerrain));
        }

        // Add raw map
        planarTerrain.gridMap.add("elevation_raw", elevationRaw);

        // Add segmentation
        planarTerrain.gridMap.add("segmentation");
        planeDecompositionPipeline_->getSegmentation(planarTerrain.gridMap.get("segmentation"));
        if (debug)
        {
          ROS_INFO("planarTerrain.gridMap it has this layers");
          layers = planarTerrain.gridMap.getLayers();
          for (const auto &layer : layers)
          {
            ROS_INFO("- %s", layer.c_str());
          }
        }
        grid_map_msgs::GridMap outputMessage;
        grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap, outputMessage);
        filteredmapPublisher_.publish(outputMessage);

        const double lineWidth = 0.005; // [m] RViz marker size
        boundaryPublisher_.publish(convertBoundariesToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                                 planarTerrain.gridMap.getTimestamp(), lineWidth));
        insetPublisher_.publish(convertInsetsToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                          planarTerrain.gridMap.getTimestamp(), lineWidth));

        iter++;

        if (iter % 40 == 0)
        {

          std::stringstream infoStream;
          infoStream << "\n########################################################################\n";
          infoStream << "The benchmarking is computed over " << iter << " iterations. \n";
          infoStream << "PlaneExtraction Benchmarking    : Average time [ms], Max time [ms]\n";
          auto printLine = [](std::string name, const convex_plane_decomposition::Timer &timer)
          {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2);
            ss << "\t" << name << "\t: " << std::setw(17) << timer.getAverageInMilliseconds() << ", " << std::setw(13)
               << timer.getMaxIntervalInMilliseconds() << "\n";
            return ss.str();
          };
          infoStream << printLine("Pre-process        ", planeDecompositionPipeline_->getPrepocessTimer());
          infoStream << printLine("Sliding window     ", planeDecompositionPipeline_->getSlidingWindowTimer());
          infoStream << printLine("Contour extraction ", planeDecompositionPipeline_->getContourExtractionTimer());
          infoStream << printLine("Post-process       ", planeDecompositionPipeline_->getPostprocessTimer());
          std::cerr << infoStream.str() << std::endl;
        }
        if (isprint)
        {
          ROS_INFO("Translation: [%.2f, %.2f, %.2f]",
                   transformStamped.transform.translation.x,
                   transformStamped.transform.translation.y,
                   transformStamped.transform.translation.z);
          ROS_INFO("Rotation: [%.2f, %.2f, %.2f, %.2f]",
                   transformStamped.transform.rotation.x,
                   transformStamped.transform.rotation.y,
                   transformStamped.transform.rotation.z,
                   transformStamped.transform.rotation.w);
        }
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Failed to lookup transform: %s", ex.what());
    }

    rate.sleep();
  }

  return 0;
}
