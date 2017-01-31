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


#ifndef ROUTE_H
#define ROUTE_H

#include <memory>
#include <vector>

#include <tuw_geometry/linesegment2d.h>
#include <tuw_geometry/point2d.h>
#include <tuw_geometry/pose2d.h>
#include <tuw_route/waypoint/waypoint.h>

namespace tuw {

class Route;
using RoutePtr      = std::shared_ptr<       Route >;
using RouteConstPtr = std::shared_ptr< const Route >;

class Route {
    //special class member functions
    public   : Route           ();
    public   : virtual ~Route  ()             = default;
    public   : Route           (const Route&) = default;
    public   : Route& operator=(const Route&) = default;
    public   : Route           (Route&&)      = default;
    public   : Route& operator=(Route&&)      = default;
    
    // functions
    public   : void all_visited          ();  ///< marks all waypoints as visited and clears active ones
    public   : void compute_line_segments();
    public   : Waypoint waypoint_relative(std::size_t i, const Pose2D &pose) const;
    public   : void               resize ( std::size_t n );
    public   : std::size_t          size () const;
    public   : const Waypoint      &goal () const;
    public   : Waypoint            &goal ();
    public   : const Waypoint      &waypoint ( std::size_t n ) const;
    public   : Waypoint            &waypoint ( std::size_t n );
    public   : const Waypoint      &active_waypoint ( std::size_t n ) const;
    public   : Waypoint            &active_waypoint ( std::size_t n );
    public   : const LineSegment2D &active_line_segment ( std::size_t n ) const;
    public   : LineSegment2D       &active_line_segment ( std::size_t n );
    public   : const LineSegment2D &line_segment ( std::size_t n ) const;
    public   : LineSegment2D       &line_segment ( std::size_t n );
    public   : int nearest_line_segment        ( const double& x, const double& y, double &distanceSqr ) const;  ///< @return idx belongs to the global idx, -1 if no active segment exists
    public   : int nearest_active_line_segment ( const double& x, const double& y, double& distanceSqr ) const;  ///< @return idx belongs to the global idx, -1 if no active segment
    public   : int nearest_active_line_segment ( const double& x, const double& y, double& distanceSqr, int& _idxPrevGuarantee ) const;  ///< @return idx belongs to the global idx, -1 if no active segmentexists
    
    public   : std::vector<tuw::Point2D>& toPoint2DWaypointVec();
    public   : std::vector<tuw::Point2D>& toPoint2DActiveWaypointVec();
    
    // variables
    public   : bool hasChanged;
    public   : std::vector<Waypoint     > waypoints_;
    public   : std::vector<LineSegment2D> line_segments_;
    public   : std::vector<int          > active_idx_;  ///< To active elements waypoints and line_segments
    
    private  : std::vector<tuw::Point2D> Point2DWaypointVec;
    private  : std::vector<tuw::Point2D> Point2DActiveWaypointVec;
};

}
#endif // ROUTE_H


