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


#include <tuw_waypoint_to_spline/tuw_waypoint_to_spline.h>
#include <float.h>

using namespace Eigen;
using namespace tuw;
using namespace std;

void Waypoint2Spline::fitSpline() {

    Eigen::MatrixXd dataPts( 3, points_[0].size() );
    for ( size_t l = 0; l < points_[0].size(); ++l ) {
        dataPts ( 0,l ) = points_[0][l];
        dataPts ( 1,l ) = points_[1][l];
        dataPts ( 2,l ) = points_[2][l];
    }

    int knotsSize = dataPts.cols();
    Eigen::MatrixXd dataPtsXY ( 2, knotsSize );
    
    for ( int i = 0; i < 2; ++i ) {
        for ( int j = 0; j < knotsSize; ++j ) {
            dataPtsXY   ( i,j ) = dataPts ( i,j );
        }
    }
    Spline3d::KnotVectorType knots;
    Eigen::ChordLengths ( dataPts, knots );
    spline_ = make_shared<Spline3d> ( SplineFitting< Spline3d >::Interpolate               ( dataPts, DenseIndex ( std::min<int> ( knotsSize - 1, 3 ) ), knots ) );
}



