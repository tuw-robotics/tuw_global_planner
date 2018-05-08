/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef TUW_SEGMENS_TO_PATH_H
#define TUW_SEGMENS_TO_PATH_H

// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tuw_geometry/pose2d.h>
#include <tuw_nav_msgs/route_segments.h>

namespace tuw
{
/**
 * class to cover the ros communication
 **/
class Segments2Path
{
public:
  Segments2Path(ros::NodeHandle &n);  /// Constructor
  void publishPath();
  void publishSegments();

private:
  ros::NodeHandle n_;            /// node handler to the root node
  ros::NodeHandle n_param_;      /// node handler to the current node
  ros::Publisher pub_path_;      /// publisher for the path waypoints
  ros::Publisher pub_segments_;  /// publisher for the segements

  /// ROS shared parameters
  std::string global_frame_id_;
  std::string file_;
  bool once_;
  double waypoints_distance_;
  double sample_distance_;
  bool update_header_timestamp_;

  nav_msgs::Path msg_path_;
  tuw::ros_msgs::RouteSegments msg_segments_;
  std::vector<Pose2D> waypoints_;
  void readSegments(const std::string &segment_file);
};
}

#endif  // TUW_SEGMENS_TO_PATH_H
