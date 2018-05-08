/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#ifndef ROUTE_CONVERTER_ROS_HPP
#define ROUTE_CONVERTER_ROS_HPP

#include <memory>

#include <tuw_route/route_manager.h>
#include <tuw_route_msgs/Route.h>

namespace tuw
{
class RouteConverterRos
{
public:
  static void toRosMsg(const RouteManager& _routeManager, tuw_route_msgs::Route& _msg)
  {
    _msg.routeWaypointSampleDist = _routeManager.routeWaypointSampleDist_;
    _msg.routeActiveArcLen = _routeManager.waypointActiveArcLen_;
    _msg.routeMaxDeviation = _routeManager.routeMaxDeviation_;
    _msg.routeIsVisitedMaxAngle = _routeManager.visitedWaypointMinDAngle_;
    _msg.waypoints.resize(_routeManager.waypoints_.size());
    for (size_t i = 0; i < _routeManager.waypoints_.size(); ++i)
    {
      const auto& waypointI = _routeManager.waypoints_[i];
      auto& msgWaypointI = _msg.waypoints[i];
      msgWaypointI.pose.x = waypointI.pose.x();
      msgWaypointI.pose.y = waypointI.pose.y();
      msgWaypointI.pose.theta = waypointI.pose.theta();
      msgWaypointI.state = waypointI.state;
    }
    _msg.waypointActiveIdx = _routeManager.active_idx_;
  }

  static void fromRosMsg(const tuw_route_msgs::Route& _msg, RouteManager& _routeManager)
  {
    _routeManager.routeWaypointSampleDist_ = _msg.routeWaypointSampleDist;
    _routeManager.waypointActiveArcLen_ = _msg.routeActiveArcLen;
    _routeManager.routeMaxDeviation_ = _msg.routeMaxDeviation;
    _routeManager.visitedWaypointMinDAngle_ = _msg.routeIsVisitedMaxAngle;

    _routeManager.waypoints_.resize(_msg.waypoints.size());
    for (size_t i = 0; i < _msg.waypoints.size(); ++i)
    {
      auto& waypointI = _routeManager.waypoints_[i];
      const auto& msgWaypointI = _msg.waypoints[i];
      waypointI.pose.x() = msgWaypointI.pose.x;
      waypointI.pose.y() = msgWaypointI.pose.y;
      waypointI.pose.theta() = msgWaypointI.pose.theta;
      waypointI.state = msgWaypointI.state;
    }
    _routeManager.active_idx_ = _msg.waypointActiveIdx;
    _routeManager.compute_line_segments();
    _routeManager.computeWaypointsDistanceToGoal();
    _routeManager.hasChanged = false;
  }
};
}
#endif  // ROUTE_CONVERTER_ROS_HPP
