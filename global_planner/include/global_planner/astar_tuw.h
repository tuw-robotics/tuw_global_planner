/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef _ASTAR_TUW_H
#define _ASTAR_TUW_H

#include <global_planner/planner_core.h>
#include <global_planner/heuristics.h>
#include <global_planner/expander.h>
#include <vector>
#include <algorithm>

namespace global_planner
{
class AStarTuwExpansion : public Expander
{
public:
  AStarTuwExpansion(PotentialCalculator* p_calc, int nx, int ny, Heuristics* hx);
  bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                           float* potential);

private:
  template <class T, class S, class C>
  void clearpq(std::priority_queue<T, S, C>& q)
  {
    q = std::priority_queue<T, S, C>();
  }
  class Index
  {
  public:
    Index(int a, float b, float c)
    {
      i = a;
      cost = b;
      dist = c;
    }
    int i;
    float cost;
    float dist;
  };

  struct greater1
  {
    bool operator()(const Index& a, const Index& b) const
    {
      return a.cost > b.cost;
    }
  };
  void calcPotentialAndAddCandidate(unsigned char* costs, float* potential, Index lastNode, int index, int end_x,
                                    int end_y, int start_x, int start_y, bool diogonal);
  std::priority_queue<Index, std::vector<Index>, greater1> queue_;
  Heuristics* hx_;
};
}  // end namespace global_planner
#endif
