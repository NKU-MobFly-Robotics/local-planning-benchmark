#ifndef LOCAL_PLANNING_BENCHMARK_OBS_DIST_CALCULATOR_H
#define LOCAL_PLANNING_BENCHMARK_OBS_DIST_CALCULATOR_H

#include <costmap_2d/costmap_2d_ros.h>
#include <ceres/cubic_interpolation.h>
#include <opencv2/opencv.hpp>

namespace move_base_benchmark
{
// A helper class implemented for calculating the distance between the robot and the closest obstacle
class ObsDistCalculator
{
public:
  ObsDistCalculator();
  ~ObsDistCalculator();

  double compute(costmap_2d::Costmap2DROS* costmap_ros) const;
};

}  // namespace move_base_benchmark

#endif  // LOCAL_PLANNING_BENCHMARK_OBS_DIST_CALCULATOR_H