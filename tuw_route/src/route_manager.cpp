/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#include <math.h>

#include <tuw_route/route_manager.h>

#include <tuw_geometry/pose2d.h>

using namespace tuw;
using namespace std;

RouteManager:: RouteManager() : routeWaypointSampleDist_(0.5) {
}

void RouteManager::update ( const Pose2D& _agentPose ) {
    agentPose_ = _agentPose;
    updateWaypoints();
}

void RouteManager::loadRoute ( const std::vector< Pose2D >& _pointsSeq ) {
    
    waypoints_.clear();
    if(_pointsSeq.empty()){ return; }
    
    waypoints_.push_back(Waypoint());
    waypoints_.back().pose = _pointsSeq[0];     
    waypoints_.back().state = Waypoint::STATE_AHEAD;
    double segmentLength = 0;
    for(size_t i = 1; i < _pointsSeq.size(); i++){
	
	double route_ds = routeWaypointSampleDist_ - segmentLength;
	
	LineSegment2D lineSegment(Point2D(_pointsSeq[i-1].x(), _pointsSeq[i-1].y()), 
			          Point2D(_pointsSeq[ i ].x(), _pointsSeq[ i ].y()));
	Point2D p0 = lineSegment.p0();
	Point2D p1 = lineSegment.p1();
	segmentLength  += lineSegment.length();
	
	while ( segmentLength > route_ds ){
	    double segmentRatio = route_ds / segmentLength;
	    waypoints_.push_back(Waypoint());
	    waypoints_.back().pose = Pose2D( p0.x() + (p1.x() - p0.x() ) * segmentRatio,
	                                             p0.y() + (p1.y() - p0.y() ) * segmentRatio,
	                                            lineSegment.angle() ); 
	    waypoints_.back().state = Waypoint::STATE_AHEAD;
	    Waypoint& lastRouteWaypoint = waypoints_.back();
	    p0 = Point2D(lastRouteWaypoint.pose.x(), lastRouteWaypoint.pose.y());
	    segmentLength -= route_ds;
	    route_ds = routeWaypointSampleDist_;
	}
    }
    waypoints_.push_back(Waypoint());
    waypoints_.back().pose = _pointsSeq.back();     
    waypoints_.back().state = Waypoint::STATE_AHEAD;
    
    compute_line_segments();
    computeWaypointsDistanceToGoal();
    
    hasChanged = true;
}

void RouteManager::computeWaypointsDistanceToGoal() {
    double d_prev = 0;
    if ( waypoints_.size() == 0 ) { return; }
    waypoints_.back().distanceToGoal = 0;
    for(size_t i = line_segments_.size(); i > 0; i--){
	waypoints_[i-2].distanceToGoal = d_prev + line_segments_[i-1].length();
	d_prev += line_segments_[i-1].length();
    }
}

void RouteManager::updateWaypoints () {

    Pose2D& p0 = agentPose_;
    /// find neraest waypoint
    double distance_to_nearest_waypoint =  std::numeric_limits<double>::max();
    size_t nearest_way_point = std::numeric_limits<size_t>::max();
    int waypoints_left = 0;
    size_t idx_last_visited = 0;
    for ( size_t i = 0; i < size(); i++ ) {
        Waypoint &waypoint = Route::waypoint ( i );
        if ( waypoint.state != Waypoint::STATE_VISIT ) {
            waypoints_left++;
	    Point2D p_i = waypoint.pose.position();
            double d = sqrt( pow(p_i.x() - p0.x(), 2) + pow(p_i.y() - p0.y(), 2) );
            if ( d < distance_to_nearest_waypoint ) {
                double a0 = waypoint.pose.theta();
                double a1 = p0.theta() + M_PI/2.;
                double da = fabs ( atan2 ( sin ( a0-a1 ), cos ( a0-a1 ) ) ); /// unsigned minimal delta angle difference
                if ( da <  ( visitedWaypointMinDAngle_ * M_PI ) / 180.0 ) {
                    distance_to_nearest_waypoint = d;
                    nearest_way_point = i;
                }
            }
        }
        else{ idx_last_visited = i; }
    }
    if ( waypoints_left > 1 ) {
        if ( nearest_way_point == size()-1 ) {
            all_visited();
        } else {
            /// update waypoints state
            if ( distance_to_nearest_waypoint < routeMaxDeviation_ ) {
                double active_distance_start = -1;
                for ( size_t i = idx_last_visited; i < size(); i++ ) {
                    Waypoint &waypoint = Route::waypoint ( i );
                    if ( i + 1 < nearest_way_point ) {// keeping one waypoint active behind the agent
                        waypoint.state = Waypoint::STATE_VISIT;
                    } else {
                        if ( active_distance_start < 0 ) {
                            active_idx_.clear ();
                            active_distance_start = waypoint.distanceToGoal;
                        }
                        double d = active_distance_start - waypoint.distanceToGoal;
                        if( d < waypointActiveArcLen_ ){
                            waypoint.state = Waypoint::STATE_ACTIVE;
                            active_idx_.push_back ( i );
                        } else {
                            waypoint.state = Waypoint::STATE_AHEAD;
                        }
                    }
                }
            }
        }
    }    
}

//TODO: intelligent A-Star checking for route
// void RouteManager::disable_unreachable_waypoints(){
//     size_t i = 0;
//     bool found_wall = false;
//     for (; i < active_idx_.size(); i++ ) {
//         const ShmFw::LineSegment2D<double> &line = line_segments_[active_idx_[i]];
// 	if(obstacle_map_->minObstacleDistAlongLine(line.x1(), line.y1(), line.x2(), line.y2()) < 0.05){ found_wall = true; break; }
//     }
//     if(!found_wall){return;}
//     size_t erase_first = fmax(0, i+1  -2); i = fmax(0, i+1-1  -2);//TODO: last -2 is here because if agent goes too close to obstacle, it will not see it anymore
//     for (; i < active_idx_.size(); i++ ) { waypoint (active_idx_[i]).state = dsa::Waypoint::STATE_AHEAD; }
//     active_idx_.erase(active_idx_.begin() + erase_first, active_idx_.end());
//     
//     if(active_idx_.size() > 0){ active_idx_.pop_back(); }
// }