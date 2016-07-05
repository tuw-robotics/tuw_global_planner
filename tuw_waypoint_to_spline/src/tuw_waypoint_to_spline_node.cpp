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
#include <vector>

using namespace tuw;
using namespace std;

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "tuw_waypoint_to_spline" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    
    ros::Rate r(5);
    Waypoint2SplineNode waypoint2SplineNode ( n );
    r.sleep();
	
    waypoint2SplineNode.fitSpline();
    waypoint2SplineNode.compTime = ros::Time::now();
    
    while ( ros::ok() ) {
	
        waypoint2SplineNode.publishSplineParam();

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
Waypoint2SplineNode::Waypoint2SplineNode ( ros::NodeHandle & n )
    : Waypoint2Spline(), 
    n_ ( n ), 
    n_param_ ( "~" ){
	
     n_param_.getParam("global_frame", global_frame_id);
	
    size_t i = 0;
    vector< vector<double> > inputData(1, vector<double>(0,0));
    while( n_param_.getParam("pt_array_dim" + to_string(i), inputData[i] ) ) { inputData.resize(inputData.size()+1); i++; }
    inputData.resize(inputData.size()-1);
    
    dataPts_ = Eigen::MatrixXd ( 3, inputData[0].size() );
    for(size_t k = 0; k < 3; ++k) {
	for(size_t l = 0; l < inputData[0].size(); ++l) {
	    dataPts_(k,l) = inputData[k][l];
	}
    }
    pubSplineData_    = n.advertise<tuw_spline_msgs::Spline>("path_spline"  , 1);
    
//     reconfigureFnc_ = boost::bind ( &Gui2IwsNode::callbackConfigBlueControl, this,  _1, _2 );
//     reconfigureServer_.setCallback ( reconfigureFnc_ );
}

// void Gui2IwsNode::callbackConfigBlueControl ( tuw_teleop::Gui2IwsConfig &config, uint32_t level ) {
//     ROS_DEBUG ( "callbackConfigBlueControl!" );
//     config_ = config;
//     init();
// }


void Waypoint2SplineNode::publishSplineParam() {
    
    
    Eigen::MatrixXd vKnots = spline_->knots();
    Eigen::MatrixXd mCtrls = spline_->ctrls();
    
    tuw_spline_msgs::Spline ss;
    ss.header.stamp = compTime;
    ss.header.frame_id = global_frame_id;
    ss.knots.resize(vKnots.cols());
    for( int i = 0; i < vKnots.cols(); ++i) { 
	ss.knots[i] = vKnots(i); 
    }
    ss.ctrls.resize(mCtrls.rows());
    for( int i = 0; i < mCtrls.rows(); ++i) { 
	ss.ctrls[i].val.resize(mCtrls.cols());
	for( int j = 0; j < mCtrls.cols(); ++j) { 
	    ss.ctrls[i].val[j] = mCtrls(i,j); 
	}
    }
    pubSplineData_.publish(ss);
}
