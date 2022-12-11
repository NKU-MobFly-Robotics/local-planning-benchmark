/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "move_base_benchmark/obs_dist_calculator.h"

namespace move_base_benchmark {

double ObsDistCalculator::compute(costmap_2d::Costmap2DROS* const costmap_ros) {
  // locking should be done in MoveBase
  const costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  const double resolution = costmap->getResolution();
  const double origin_x = costmap->getOriginX();
  const double origin_y = costmap->getOriginY();
  const unsigned char* charmap = costmap->getCharMap();

  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros->getRobotPose(robot_pose);

  // compute Euclidean distance map
  cv::Mat gridMapImage(size_y, size_x, CV_8UC1);

  uchar* uchar_ptr = gridMapImage.ptr<uchar>(0);
  for (int i = 0; i < gridMapImage.rows * gridMapImage.cols; ++i) {
    if (charmap[i] == costmap_2d::LETHAL_OBSTACLE) {
      uchar_ptr[i] = 0;
    } else {
      uchar_ptr[i] = 255;
    }
  }

  // calculate the educlidean distance transform via OpenCV distanceTransform
  // function
  cv::Mat distanceFieldImage;
  cv::distanceTransform(gridMapImage, distanceFieldImage, cv::DIST_L2,
                        cv::DIST_MASK_PRECISE);

  float* float_ptr = distanceFieldImage.ptr<float>(0);
  std::vector<double> distmap(size_x * size_y);
  for (int i = 0; i < distanceFieldImage.rows * distanceFieldImage.cols; ++i) {
    distmap[i] = static_cast<double>(float_ptr[i]) * resolution;
  }

  // ceres bicubic interpolation
  ceres::Grid2D<double, 1, false> grid(distmap.data(), 0, size_x, 0, size_y);
  ceres::BiCubicInterpolator<ceres::Grid2D<double, 1, false>> interpolator(
      grid);

  double r = (robot_pose.pose.position.x - origin_x) / resolution;
  double c = (robot_pose.pose.position.y - origin_y) / resolution;

  double dist = 0.0;
  interpolator.Evaluate(r, c, &dist);
  return dist;
}

}  // namespace move_base_benchmark
