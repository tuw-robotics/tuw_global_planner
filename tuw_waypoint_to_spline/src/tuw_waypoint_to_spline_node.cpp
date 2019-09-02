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

#include <tuw_waypoint_to_spline/tuw_waypoint_to_spline_node.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <tf/LinearMath/Matrix3x3.h>

using namespace tuw;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tuw_waypoint_to_spline");  /// initializes the ros node with default name
  ros::NodeHandle n;

  ros::Rate r(5);
  Waypoint2SplineNode node(n);
  r.sleep();

  while (ros::ok())
  {

    /// calls all callbacks waiting in the queue
    ros::spinOnce();

    /// sleeps for the time remaining to let us hit our publish rate
    r.sleep();
  }
  return 0;
}

/**
 * Constructor
 **/
Waypoint2SplineNode::Waypoint2SplineNode(ros::NodeHandle &n) : Waypoint2Spline(), n_(n), n_param_("~")
{
  n_param_.param<std::string>("global_frame", global_frame_id_, "/map");
  std::string path_file;
  n_param_.getParam("path_file", path_file);
  n_param_.param<int>("minimum_number_of_points", minimum_number_of_points_, 5);
  n_param_.param<double>("min_waypoint_distance", min_waypoint_distance_, 0.0);  /// minimum distance between waypoints
                                                                                 /// to used as spline knots, 0 means
                                                                                 /// all waypoints are used
  n_param_.param<string>("path_tmp_file", path_tmp_file_, "/tmp/waypoints_to_spline.yaml");
  spline_msg_.header.seq = 0;
  if (!path_file.empty())
  {
    constructSplineFromFile(path_file);
  }

  pubSplineData_ = n.advertise<tuw_nav_msgs::Spline>("path_spline", 1);
  sub_path_ = n.subscribe("path", 1, &Waypoint2SplineNode::callbackPath, this);
  service_server = n.advertiseService("path_to_spline", &Waypoint2SplineNode::pathToSplineCall, this);
  //     reconfigureFnc_ = boost::bind ( &Gui2IwsNode::callbackConfigBlueControl, this,  _1, _2 );
  //     reconfigureServer_.setCallback ( reconfigureFnc_ );
}

// void Gui2IwsNode::callbackConfigBlueControl ( tuw_teleop::Gui2IwsConfig &config, uint32_t level ) {
//     ROS_DEBUG ( "callbackConfigBlueControl!" );
//     config_ = config;
//     init();
// }

tuw_nav_msgs::Spline Waypoint2SplineNode::pathToSpline(const nav_msgs::Path &path) {
    if (path.poses.size() > 0)
    {
        size_t repeatInitEndPt = 1;
        double deltaXInitEndPt = 0.001;
        double distFreeEnd = 1.5;
        ROS_INFO("constructSplineFromPath");
        for (int i = 0; i < 3; i++)
        {
            points_[i].clear();
            points_[i].reserve(path.poses.size() + 2 * repeatInitEndPt);
            derivatives_[i].resize(2);
        }

        tf::StampedTransform tf_map2base;
        try
        {
            tf_listener_.lookupTransform("map", tf::resolve(n_.getNamespace(), "base_link"), ros::Time(0), tf_map2base);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.1).sleep();
            throw runtime_error{ex.what()};
        }
        double roll, pitch, yaw;
        tf::Quaternion qt;
        tf::Matrix3x3(tf_map2base.getRotation()).getRPY(roll, pitch, yaw);

        const geometry_msgs::Pose &pose = path.poses[0].pose;
        points_[0].push_back(pose.position.x);
        points_[1].push_back(pose.position.y);
        points_[2].push_back(0);

        for (size_t rep = 0; rep < repeatInitEndPt; ++rep)
        {
            points_[0].push_back(points_[0].back() + deltaXInitEndPt * cos(yaw));
            points_[1].push_back(points_[1].back() + deltaXInitEndPt * sin(yaw));
            points_[2].push_back(0);
        }
        size_t idxEndBeforeLast;
        double dFromEnd = 0;
        for (size_t i = path.poses.size(); i > 0; --i)
        {
            const geometry_msgs::Pose &pose = path.poses[i - 1].pose;
            if (i > 1)
            {
                double dx = path.poses.back().pose.position.x - pose.position.x;
                double dy = path.poses.back().pose.position.y - pose.position.y;
                dFromEnd += sqrt(dx * dx + dy * dy);
                if (dFromEnd > distFreeEnd)
                {
                    idxEndBeforeLast = i - 1;
                    break;
                }
            }
            else
            {
                idxEndBeforeLast = i;
            }
        }
        bool initPart = true;
        double dFromStart = 0;
        for (size_t i = 1; i < path.poses.size(); i++)
        {
            const geometry_msgs::Pose &pose = path.poses[i].pose;
            if (initPart)
            {
                double dx = points_[0].back() - pose.position.x;
                double dy = points_[1].back() - pose.position.y;
                dFromStart += sqrt(dx * dx + dy * dy);
                if (dFromStart < distFreeEnd)
                    continue;
                else
                {
                    initPart = false;
                }
            }
            if (i < idxEndBeforeLast)
            {
                double dx = points_[0].back() - pose.position.x;
                double dy = points_[1].back() - pose.position.y;
                double d = sqrt(dx * dx + dy * dy);
                if (d < min_waypoint_distance_)
                {
                    continue;
                }
            }
            else
            {
                if (i < path.poses.size() - 1)
                {
                    continue;
                }
            }
            points_[0].push_back(pose.position.x);
            points_[1].push_back(pose.position.y);
            points_[2].push_back(0);
        }
        qt = tf::Quaternion(path.poses.back().pose.orientation.x, path.poses.back().pose.orientation.y,
                            path.poses.back().pose.orientation.z, path.poses.back().pose.orientation.w);
        tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
        for (size_t rep = 0; rep < repeatInitEndPt; ++rep)
        {
            points_[0].push_back(points_[0].back() + deltaXInitEndPt * cos(yaw));
            points_[1].push_back(points_[1].back() + deltaXInitEndPt * sin(yaw));
            points_[2].push_back(0);
        }
    }

    if (!path_tmp_file_.empty())
    {
        YAML::Node waypoints_yaml;
        waypoints_yaml["x"] = points_[0];
        waypoints_yaml["y"] = points_[1];
        waypoints_yaml["o"] = points_[2];
        std::ofstream fout(path_tmp_file_.c_str());
        fout << waypoints_yaml;
        ROS_INFO("%s: %s", "created waypoint file: ", path_tmp_file_.c_str());
    }
    size_t N = points_[0].size();
    ROS_INFO("The path contained: %zu points", N);
    if (N < (size_t)minimum_number_of_points_)
    {
        ROS_ERROR("The path must contain at least: %i points", minimum_number_of_points_);
        std::stringstream string_stream;
        string_stream << "The path must contain at least: " << minimum_number_of_points_ << " points";
        throw runtime_error{string_stream.str().c_str()};
    }

    tuw_nav_msgs::Spline spline_msg = constructSplineMsg();
    spline_msg.header.frame_id = path.header.frame_id;
    spline_msg.header.stamp = path.header.stamp;
    spline_msg.header.seq = path.header.seq;


    return spline_msg;
}

void Waypoint2SplineNode::constructSplineFromFile(const std::string &file)
{
  ROS_INFO("constructSplineFromFile: %s", file.c_str());
  YAML::Node waypoints_yaml = YAML::LoadFile(file);
  points_[0] = waypoints_yaml["x"].as<std::vector<double> >();
  points_[1] = waypoints_yaml["y"].as<std::vector<double> >();
  points_[2] = waypoints_yaml["o"].as<std::vector<double> >();

  size_t N = points_[0].size();
  ROS_INFO("The file contained: %zu points", N);
  if (N < (size_t)minimum_number_of_points_)
  {
    ROS_ERROR("The path must contain at least: %i points", minimum_number_of_points_);
    return;
  }
  if ((N > 0) && (points_[0].size() == N) && (points_[1].size() == N) && (points_[2].size() == N))
  {
    sleep(1);  // the sleep was needed

    spline_msg_ = constructSplineMsg();
    spline_msg_.header.frame_id = global_frame_id_;
    spline_msg_.header.stamp = ros::Time::now();
    spline_msg_.header.seq = spline_msg_.header.seq + 1;
  }
}

bool Waypoint2SplineNode::pathToSplineCall(tuw_waypoint_to_spline::PathToSplineRequest &req, tuw_waypoint_to_spline::PathToSplineResponse &res) {
    try {
        res.spline = pathToSpline(req.path);
        return true;
    } catch (runtime_error& error) {
        ROS_ERROR("Service Error: %s", error.what());
        return false;
    }
}

void Waypoint2SplineNode::callbackPath(const nav_msgs::Path &msg)
{
    try {
        const auto spline = pathToSpline(msg);
        publishSpline(spline);
    } catch (const runtime_error &error) {
        ROS_ERROR("Caught Exception: %s", error.what());
    }
}

tuw_nav_msgs::Spline Waypoint2SplineNode::constructSplineMsg()
{
  fitSpline();
  Eigen::MatrixXd vKnots = spline_->knots();
  Eigen::MatrixXd mCtrls = spline_->ctrls();

  tuw_nav_msgs::Spline spline;
  spline.header.seq = 0;
  spline.knots.resize(vKnots.cols());
  for (int i = 0; i < vKnots.cols(); ++i)
  {
    spline.knots[i] = vKnots(i);
  }
  spline.ctrls.resize(mCtrls.rows());
  for (int i = 0; i < mCtrls.rows(); ++i)
  {
    spline.ctrls[i].val.resize(mCtrls.cols());
    for (int j = 0; j < mCtrls.cols(); ++j)
    {
      spline.ctrls[i].val[j] = mCtrls(i, j);
    }
  }
  return spline;
}
void Waypoint2SplineNode::publishSpline(const tuw_nav_msgs::Spline& spline)
{
    pubSplineData_.publish(spline);
}
