/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
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


#include <tuw_route/route.h>

using namespace tuw;
using namespace std;

Route::Route (): hasChanged(true) {}

void Route::resize ( std::size_t n ) {
    waypoints_.resize ( n );
}

const Waypoint &Route::waypoint ( std::size_t n ) const {
    return waypoints_[n];
}
Waypoint &Route::waypoint ( std::size_t n ) {
    return waypoints_[n];
}
const Waypoint &Route::goal ( ) const {
    return waypoints_.back();
}
Waypoint &Route::goal () {
    return waypoints_.back();
}
const Waypoint &Route::active_waypoint ( std::size_t n ) const {
    return waypoints_[active_idx_[n]];
}
Waypoint &Route::active_waypoint ( std::size_t n ) {
    return waypoints_[active_idx_[n]];
}
const LineSegment2D &Route::active_line_segment ( std::size_t n ) const {
    return line_segments_[active_idx_[n]];
}
LineSegment2D &Route::active_line_segment ( std::size_t n ) {
    return line_segments_[active_idx_[n]];
}
const LineSegment2D &Route::line_segment ( std::size_t n ) const {
    return line_segments_[n];
}
LineSegment2D &Route::line_segment ( std::size_t n ) {
    return line_segments_[n];
}
Waypoint Route::waypoint_relative ( size_t i, const Pose2D &pose ) const {
    Waypoint waypoint = waypoints_[i];
    waypoint.pose = Pose2D( waypoints_[i].pose.x() - pose.x(), waypoints_[i].pose.y() - pose.y(), waypoints_[i].pose.theta() - pose.theta() );
    return waypoint;
}

void Route::compute_line_segments() {
    if ( waypoints_.empty() ) {
        line_segments_.clear();
        return;
    }
    line_segments_.resize ( waypoints_.size() );
    double l = 0.01;  /// length dummy segment
    for ( size_t i = 0; i < line_segments_.size()-1; i++ ) {
        Pose2D &pt0 = waypoint ( i ).pose;
        Pose2D &pt1 = waypoint ( i+1 ).pose;
        line_segments_[i] = ::LineSegment2D ( pt0.x(), pt0.y(), pt1.x(), pt1.y() );
    }
    Pose2D &p = goal().pose;
    line_segments_.back() = ::LineSegment2D (p.x(), p.y(), p.x() + p.theta_cos()*l, p.y() + p.theta_sin()*l );
}

std::size_t Route::size() const {
    return waypoints_.size();
}

int Route::nearest_active_line_segment ( const double& x, const double& y, double &distanceSqr ) const {
    Point2D p(x, y);
    distanceSqr = std::numeric_limits<double>::max();
    int idx = -1;
    double dx_dummy, dy_dummy;
    for ( size_t i = 0; i < active_idx_.size(); i++ ) {
        const ::LineSegment2D &line = line_segments_[active_idx_[i]];
        const double dSqr = line.distanceSqrTo ( p, dx_dummy, dy_dummy );
        if ( dSqr < distanceSqr ) {
            distanceSqr = dSqr;
            idx = active_idx_[i];
        }
    }
    return idx;
}

int Route::nearest_active_line_segment ( const double& x, const double& y, double &distanceSqr, int& _idxPrevGuarantee ) const {
    Point2D p(x, y);
    distanceSqr = std::numeric_limits<double>::max();
    int idx = -1;
    double dx_dummy, dy_dummy;
    for ( size_t i = _idxPrevGuarantee; i < active_idx_.size(); i++ ) {
        const ::LineSegment2D &line = line_segments_[active_idx_[i]];
        const double dSqr = line.distanceSqrTo ( p, dx_dummy, dy_dummy );
        if ( dSqr < distanceSqr ) {
            distanceSqr = dSqr;
            idx = active_idx_[i];
        } else { break; }
        if (i > _idxPrevGuarantee) { _idxPrevGuarantee++; }
    }
    return idx;
}

void Route::all_visited() {
  for(size_t i = 0; i < waypoints_.size(); i++){
    waypoints_[i].state = Waypoint::STATE_VISIT;
  }
  active_idx_.clear();
}
int Route::nearest_line_segment ( const double& x, const double& y, double &distanceSqr ) const {
    distanceSqr = std::numeric_limits<double>::max();
    int idx = -1;
    double dx_dummy, dy_dummy;
    for ( size_t i = 0; i < line_segments_.size(); i++ ) {
        const ::LineSegment2D &line = line_segments_[i];
        const double dSqr = line.distanceSqrTo ( Point2D(x, y), dx_dummy, dy_dummy );
        if ( dSqr < distanceSqr ) {
            distanceSqr = dSqr;
            idx = i;
        }
    }
    return idx;
}

vector<Point2D>& Route::toPoint2DWaypointVec() {
    if ( hasChanged ) {
	Point2DWaypointVec.resize(waypoints_.size());
	for(size_t i = 0; i < waypoints_.size(); i++){ Point2DWaypointVec[i] = waypoints_[i].pose.position(); }
	hasChanged = false;
    }
    return Point2DWaypointVec;
}

vector<Point2D>& Route::toPoint2DActiveWaypointVec(){
    Point2DActiveWaypointVec.resize(active_idx_.size());
    for(size_t i = 0; i < active_idx_.size(); i++){ Point2DActiveWaypointVec[i] = active_waypoint(i).pose.position(); }
    return Point2DActiveWaypointVec;
}