/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2015 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#ifndef TUW_PATH_TO_SPLINE_NODE_H
#define TUW_PATH_TO_SPLINE_NODE_H

#include <tuw_waypoint_to_spline/tuw_waypoint_to_spline.h>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_waypoint_to_spline_msgs/PathToSpline.h>
#include <tuw_nav_msgs/Spline.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

namespace tuw
{
/**
 * class to cover the ros communication
 **/


class Waypoint2SplineNode : public Waypoint2Spline
{
public:
  Waypoint2SplineNode(ros::NodeHandle &n);  /// Constructor
  void publishSpline(const tuw_nav_msgs::Spline& spline);

private:
  ros::NodeHandle n_;             /// node handler to the root node
  ros::NodeHandle n_param_;       /// node handler to the current node
  ros::Publisher pubSplineData_;  /// publisher for the motion commands
  ros::Subscriber sub_path_;      /// Subscriber to the laser measurements
  ros::Subscriber sub_path;       /// Subscriber to the laser measurements
  ros::ServiceServer service_server;

private:
  tf::TransformListener tf_listener_;

  std::string global_frame_id_;
  tuw_nav_msgs::Spline spline_msg_;
  double min_waypoint_distance_;
  std::string path_tmp_file_;
  int minimum_number_of_points_;
  bool pathToSplineCall(tuw_waypoint_to_spline_msgs::PathToSplineRequest& req, tuw_waypoint_to_spline_msgs::PathToSplineResponse& res);

  //     dynamic_reconfigure::Server<tuw_path_to_spline::Path2SplineNodeConfig> reconfigureServer_; /// parameter server
  //     stuff
  //     dynamic_reconfigure::Server<tuw_path_to_spline::Path2SplineNodeConfig>::CallbackType reconfigureFnc_;  ///
  //     parameter server stuff
  void callbackPath(const nav_msgs::Path &);  /// callback function to execute on path msg

  tuw_nav_msgs::Spline pathToSpline(const nav_msgs::Path& path);
  void constructSplineFromFile(const std::string &file);
  tuw_nav_msgs::Spline constructSplineMsg();
};
}

#endif  // TUW_PATH_TO_SPLINE_NODE_H
