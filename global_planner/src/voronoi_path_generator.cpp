#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <global_planner/voronoi_path_generator.h>
#include <global_planner/thinning.h>
#include <memory>
#include <opencv/cv.hpp>
#include <queue>
#include <string>

using namespace cv;

namespace global_planner
{
VoronoiPathGenerator::VoronoiPathGenerator()
{
}

void VoronoiPathGenerator::prepareMap(const Mat& _map, Mat& _smoothedMap, int blurSize)
{
  if (blurSize % 2 != 1 && blurSize != 0)
    blurSize++;

  static Mat srcMap;
  _map.convertTo(srcMap, CV_8UC1);

  for (int i = 0; i < srcMap.cols * srcMap.rows; i++)
  {
    if ((signed char)_map.data[i] < 0)
      srcMap.data[i] = 100;
  }

  _smoothedMap = srcMap;

  try
  {
    if (blurSize > 0)
    {
      cv::Size sz(blurSize, blurSize);
      cv::GaussianBlur(srcMap, srcMap, sz, 0);
      cv::GaussianBlur(srcMap, srcMap, sz, 0);
    }
  }
  catch (...)
  {
    ROS_INFO("Smoothing map failed");
  }

  cv::bitwise_not(srcMap, srcMap);
  cv::threshold(srcMap, _smoothedMap, 10, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
}

void VoronoiPathGenerator::computeDistanceField(const cv::Mat& _map, cv::Mat& _distField)
{
  cv::distanceTransform(_map, _distField, CV_DIST_L2, 3);
}

void VoronoiPathGenerator::computeVoronoiMap(const cv::Mat& _distField, cv::Mat& _voronoiMap)
{
  Mat srcMap = _distField;
  srcMap.convertTo(_voronoiMap, CV_8UC1, 0.0);

  global_planner::greyscale_thinning(srcMap, _voronoiMap);
  cv::threshold(_voronoiMap, _voronoiMap, 1, 255, CV_THRESH_BINARY);
  global_planner::sceletonize(_voronoiMap, _voronoiMap);
}
}
