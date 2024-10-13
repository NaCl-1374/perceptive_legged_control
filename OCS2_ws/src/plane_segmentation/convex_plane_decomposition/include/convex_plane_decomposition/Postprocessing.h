/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-03-05 21:50:25
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-03-10 16:37:03
 * @FilePath: /leg_perceptive_ws/src/plane_segmentation/convex_plane_decomposition/include/convex_plane_decomposition/Postprocessing.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "convex_plane_decomposition/PlanarRegion.h"

namespace convex_plane_decomposition {

struct PostprocessingParameters {
  /// Added offset in z direction to compensate for the location of the foot frame w.r.t. the elevation map
  double extracted_planes_height_offset = 0.0;

  /// Added offset in z direction added in cells that are not traversable
  double nonplanar_height_offset = 0.02;

  /// Size of the kernel creating the boundary offset. In number of pixels.
  int nonplanar_horizontal_offset = 1;

  /// Half the width of the dilation used before the smooth layer [m]
  double smoothing_dilation_size = 0.2;

  /// Half the width of the box kernel used for the smooth layer [m]
  double smoothing_box_kernel_size = 0.1;

  /// Half the width of the Gaussian kernel used for the smooth layer [m]
  double smoothing_gauss_kernel_size = 0.05;
};

class Postprocessing {
 public:
  Postprocessing(const PostprocessingParameters& parameters);

  /**
   * @param planarTerrain
   * @param elevationLayer : name of the elevation layer
   * @param planeSegmentationLayer : name of the planarity layer with planar = 1.0, non-planar = 0.0
   */
  void postprocess(PlanarTerrain& planarTerrain, const std::string& elevationLayer, const std::string& planeSegmentationLayer) const;

 private:
  void dilationInNonplanarRegions(Eigen::MatrixXf& elevationData, const Eigen::MatrixXf& planarityMask) const;
  void addHeightOffset(Eigen::MatrixXf& elevationData, const Eigen::MatrixXf& planarityMask) const;
  void addHeightOffset(std::vector<PlanarRegion>& planarRegions) const;
  void addSmoothLayer(grid_map::GridMap& gridMap, const Eigen::MatrixXf& elevationData, const Eigen::MatrixXf& planarityMask) const;
  void addSmoothLayer_custom(grid_map::GridMap& gridMap, const Eigen::MatrixXf& elevationData, const Eigen::MatrixXf& planarityMask) const;

  PostprocessingParameters parameters_;
};

}  // namespace convex_plane_decomposition
