#include "move_base_benchmark/obs_dist_calculator.h"

namespace move_base_benchmark
{
ObsDistCalculator::ObsDistCalculator()
{
}

ObsDistCalculator::~ObsDistCalculator()
{
}

double ObsDistCalculator::compute(costmap_2d::Costmap2DROS* costmap_ros) const
{
  // locking should be done in MoveBase
  costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
  int width = costmap->getSizeInCellsX();
  int height = costmap->getSizeInCellsY();
  double resolution = costmap->getResolution();
  double origin_x = costmap->getOriginX();
  double origin_y = costmap->getOriginY();
  const unsigned char* charmap = costmap->getCharMap();

  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros->getRobotPose(robot_pose);

  // compute Euclidean distance map
  cv::Mat gridMapImage(height, width, CV_8UC1);

  uchar* uchar_ptr = gridMapImage.ptr<uchar>(0);
  for (int i = 0; i < gridMapImage.rows * gridMapImage.cols; ++i)
  {
    if (charmap[i] == costmap_2d::LETHAL_OBSTACLE)
    {
      uchar_ptr[i] = 0;
    }
    else
    {
      uchar_ptr[i] = 255;
    }
  }

  // calculate the educlidean distance transform via OpenCV distanceTransform function
  cv::Mat distanceFieldImage;
  cv::distanceTransform(gridMapImage, distanceFieldImage, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  float* float_ptr = distanceFieldImage.ptr<float>(0);
  double* distmap = new double[width * height];
  for (int i = 0; i < distanceFieldImage.rows * distanceFieldImage.cols; ++i)
  {
    distmap[i] = static_cast<double>(float_ptr[i]) * resolution;
  }

  // ceres bicubic interpolation
  ceres::Grid2D<double, 1, false> grid(distmap, 0, width, 0, height);

  ceres::BiCubicInterpolator<ceres::Grid2D<double, 1, false> > interpolator(grid);

  double r = (robot_pose.pose.position.x - origin_x) / resolution;
  double c = (robot_pose.pose.position.y - origin_y) / resolution;

  double dist = 0.0;
  interpolator.Evaluate(r, c, &dist);

  delete distmap;

  return dist;
}

}  // namespace move_base_benchmark