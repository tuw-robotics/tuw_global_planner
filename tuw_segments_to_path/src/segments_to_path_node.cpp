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

#include <tuw_segments_to_path/segments_to_path_node.h>

#include <iostream>
#include <fstream>
#include <vector>
#include "yaml-cpp/yaml.h"

using namespace tuw;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segments_to_path");  /// initializes the ros node with default name
  ros::NodeHandle n;

  Segments2Path node(n);
  return 0;
}

/**
 * Constructor
 **/
Segments2Path::Segments2Path(ros::NodeHandle &n) : n_(n), n_param_("~")
{
  n_param_.param<std::string>("global_frame", global_frame_id_, "map");
  n_param_.getParam("file", file_);           // yaml file with semgents
  n_param_.param<bool>("once", once_, true);  // a publishs path only onece
  n_param_.param<bool>("update_header_timestamp", update_header_timestamp_, true);
  n_param_.param<double>("sample_distance", sample_distance_, 0.1);
  pub_path_ = n.advertise<nav_msgs::Path>("path", 1);
  pub_segments_ = n.advertise<tuw_nav_msgs::RouteSegments>("segments", 1);
  if (!file_.empty())
  {
    readSegments(file_);
  }

  msg_segments_.header.seq = 0;

  ros::Rate r(1);
  ros::ok();
  r.sleep();

  do
  {
    publishPath();
    publishSegments();

    /// calls all callbacks waiting in the queue
    ros::spinOnce();
    /// sleeps for the time remaining to let us hit our publish rate
    r.sleep();
  } while (ros::ok() && !once_);
}

void Segments2Path::readSegments(const std::string &segment_file)
{
  YAML::Node waypoints_yaml = YAML::LoadFile(segment_file);
  msg_segments_.set_ids(waypoints_yaml["id"].as<std::vector<unsigned int> >());
  msg_segments_.set_type(waypoints_yaml["type"].as<std::vector<unsigned int> >());
  msg_segments_.set_orientation(waypoints_yaml["orientation"].as<std::vector<unsigned int> >());
  msg_segments_.set_motion_type(waypoints_yaml["motion_type"].as<std::vector<unsigned int> >());
  msg_segments_.set_start(waypoints_yaml["start_x"].as<std::vector<double> >(),
                          waypoints_yaml["start_y"].as<std::vector<double> >(),
                          waypoints_yaml["start_theta"].as<std::vector<double> >());
  msg_segments_.set_end(waypoints_yaml["end_x"].as<std::vector<double> >(),
                        waypoints_yaml["end_y"].as<std::vector<double> >(),
                        waypoints_yaml["end_theta"].as<std::vector<double> >());
  msg_segments_.set_center(waypoints_yaml["center_x"].as<std::vector<double> >(),
                           waypoints_yaml["center_y"].as<std::vector<double> >(),
                           waypoints_yaml["center_theta"].as<std::vector<double> >());
  msg_segments_.set_level(waypoints_yaml["level"].as<std::vector<int> >());
  msg_segments_.header.frame_id = global_frame_id_;
  msg_segments_.header.stamp = ros::Time::now();

  msg_segments_.convert(msg_path_, sample_distance_);
}

void Segments2Path::publishPath()
{
  if (update_header_timestamp_)
  {
    msg_path_.header.stamp = ros::Time::now();
  }
  pub_path_.publish(msg_path_);
  msg_path_.header.seq = msg_path_.header.seq + 1;
}

void Segments2Path::publishSegments()
{
  if (update_header_timestamp_)
  {
    msg_segments_.header.stamp = ros::Time::now();
  }
  pub_segments_.publish((tuw_nav_msgs::RouteSegments &)msg_segments_);
  msg_segments_.header.seq = msg_segments_.header.seq + 1;
}
