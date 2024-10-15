#include <ros/init.h>
#include <ros/node_handle.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <convex_plane_decomposition/LoadGridmapFromImage.h>
#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_ros/ParameterLoading.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <segmented_planes_terrain_model/SegmentedPlanesTerrainModel.h>


int main(int argc, char* argv[]) {
  const std::string path(__FILE__);
  const std::string ocs2_anymal = path.substr(0, path.find_last_of("/")) + "/../../";
  const std::string terrainFolder = ocs2_anymal + "plot_terrain/data/";

  const std::string robotName = "anymal";

  ros::init(argc, argv, "perception_and_rviz_demo");
  ros::NodeHandle nodeHandle("~");
  std::cout<<nodeHandle.getNamespace()<<std::endl;
  std::string urdfString;
  ros::param::get("ocs2_anymal_description", urdfString);

    // 感知配置
  convex_plane_decomposition::PlaneDecompositionPipeline::Config perceptionConfig;
  perceptionConfig.preprocessingParameters =
      convex_plane_decomposition::loadPreprocessingParameters(nodeHandle, "preprocessing/");
  perceptionConfig.contourExtractionParameters = convex_plane_decomposition::loadContourExtractionParameters(
      nodeHandle, "contour_extraction/");
  perceptionConfig.ransacPlaneExtractorParameters = convex_plane_decomposition::loadRansacPlaneExtractorParameters(
      nodeHandle, "ransac_plane_refinement/");
  perceptionConfig.slidingWindowPlaneExtractorParameters = convex_plane_decomposition::loadSlidingWindowPlaneExtractorParameters(
      nodeHandle, "sliding_window_plane_extractor/");
  perceptionConfig.postprocessingParameters =
      convex_plane_decomposition::loadPostprocessingParameters(nodeHandle, "postprocessing/");


  // 载入地图
  const std::string elevationLayer{"elevation"};
  const std::string frameId{"world"};
  std::string terrainFile{""};
  nodeHandle.getParam("/terrain_name", terrainFile);
  double heightScale{1.0};
  nodeHandle.getParam("/terrain_scale", heightScale);
  auto gridMap = convex_plane_decomposition::loadGridmapFromImage(terrainFolder + "/" + terrainFile, elevationLayer, frameId,
                                                                  perceptionConfig.preprocessingParameters.resolution, heightScale);
  gridMap.get(elevationLayer).array() -= gridMap.atPosition(elevationLayer, {0., 0.});



  // 运行感知管线
  convex_plane_decomposition::PlaneDecompositionPipeline planeDecompositionPipeline(perceptionConfig);
  planeDecompositionPipeline.update(grid_map::GridMap(gridMap), elevationLayer);
  auto& planarTerrain = planeDecompositionPipeline.getPlanarTerrain();
  auto terrainModel = std::unique_ptr<switched_model::SegmentedPlanesTerrainModel>(new switched_model::SegmentedPlanesTerrainModel(planarTerrain));

  // Read min-max from elevation map
  const float heightMargin = 0.5;  // Create SDF till this amount above and below the map.
  const auto& elevationData = gridMap.get(elevationLayer);
  const float minValue = elevationData.minCoeffOfFinites() - heightMargin;
  const float maxValue = elevationData.maxCoeffOfFinites() + heightMargin;
  terrainModel->createSignedDistanceBetween({-1e30, -1e30, minValue}, {1e30, 1e30, maxValue});  // will project XY range to map limits


  // 在RViz中可视化
  ros::Publisher elevationmapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  ros::Publisher filteredmapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("filtered_map", 1);
  ros::Publisher boundaryPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("boundaries", 1);
  ros::Publisher insetPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("insets", 1);
  ros::Publisher distanceFieldPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("signed_distance_field", 1, true);

  // 为可视化创建点云
  const auto* sdfPtr = dynamic_cast<const switched_model::SegmentedPlanesSignedDistanceField*>(
      terrainModel->getSignedDistanceField());
  sensor_msgs::PointCloud2 pointCloud2Msg;
  if (sdfPtr != nullptr) {
    const auto& sdf = sdfPtr->asGridmapSdf();
    grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float val) { return val <= 0.0F; });
  }

  // 网格地图
  grid_map_msgs::GridMap filteredMapMessage;
  grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap, filteredMapMessage);
  grid_map_msgs::GridMap elevationMapMessage;
  grid_map::GridMapRosConverter::toMessage(gridMap, elevationMapMessage);

  // 分割
  const double lineWidth = 0.005;
  auto boundaries = convertBoundariesToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                  planarTerrain.gridMap.getTimestamp(), lineWidth);
  auto boundaryInsets = convertInsetsToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                  planarTerrain.gridMap.getTimestamp(), lineWidth);

  while (ros::ok()) {
    // 在RViz中可视化
    filteredmapPublisher.publish(filteredMapMessage);
    elevationmapPublisher.publish(elevationMapMessage);
    boundaryPublisher.publish(boundaries);
    insetPublisher.publish(boundaryInsets);

    if (sdfPtr != nullptr) {
      distanceFieldPublisher.publish(pointCloud2Msg);
    }

    ros::WallDuration(1.0).sleep();
  }

  return 0;
}
