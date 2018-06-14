
#ifndef VORONOI_PATH_GENERATOR
#define VORONOI_PATH_GENERATOR

#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <opencv/cv.h>

#define DEFAULT_MAP_NAME "voronoi_map"

namespace global_planner
{
class VoronoiPathGenerator
{
public:
  VoronoiPathGenerator();
  /**
   * @brief Smoothes the map to get better voronoi graph results
   * 
   * @param _map 
   * @param _smoothedMap 
   * @param blurSize 
   */
  void prepareMap(const cv::Mat &_map, cv::Mat &_smoothedMap, int blurSize);
  /**
   * @brief computes the distance field of a map
   * 
   * @param _map 
   * @param _distField 
   */
  void computeDistanceField(const cv::Mat &_map, cv::Mat &_distField);
  /**
   * @brief computes the voronoi map of a distance field.
   * 
   * @param _distField 
   * @param _voronoiMap 
   */
  void computeVoronoiMap(const cv::Mat &_distField, cv::Mat &_voronoiMap);
};
} // namespace global_planner

#endif
