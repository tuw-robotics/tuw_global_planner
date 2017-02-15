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


#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <cstdint>
#include <iostream>

#include <tuw_geometry/pose2d.h>

namespace tuw {

class Waypoint {

    public   : enum Validity {
      VALID = 0,
      INVALID = 1
    };
    public   : static constexpr const uint16_t STATE_VISIT = 0;
    public   : static constexpr const uint16_t STATE_ACTIVE = 1;
    public   : static constexpr const uint16_t STATE_AHEAD = 2;
    
    //special class member functions
    public   : Waypoint           ();
    public   : ~Waypoint          ()                = default;
    public   : Waypoint           (const Waypoint&) = default;
    public   : Waypoint& operator=(const Waypoint&) = default;
    public   : Waypoint           (Waypoint&&)      = default;
    public   : Waypoint& operator=(Waypoint&&)      = default;
    
    
    public   : Validity       isTypeValid() const;
    public   : std::string getStateString() const;
    
    public   : Pose2D pose;
    public   : double distanceToGoal;
    public   : uint16_t state;  ///< state STATE_ACTIVE, STATE_VIST or STATE_AHEAD
    
    public   : friend std::ostream& operator<< ( std::ostream &os, const tuw::Waypoint &o ) {
        os << "[Waypoint: pose=" << o.pose;
        os <<         ", state=" << o.getStateString();
        os <<         ", sGoal=" << o.distanceToGoal << "m";
        return os;
    }
};


}
#endif // WAYPOINT_H
