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
#include <global_planner/astar_tuw.h>
#include <global_planner/heuristics.h>
#include <costmap_2d/cost_values.h>

namespace global_planner
{
AStarTuwExpansion::AStarTuwExpansion(PotentialCalculator *p_calc, int xs, int ys, Heuristics *hx)
    : Expander(p_calc, xs, ys)
{
  hx_ = hx;
}

bool AStarTuwExpansion::calculatePotentials(unsigned char *costs, double start_x, double start_y, double end_x,
                                            double end_y, int cycles, float *potential)
{
  int cycle = 0;
  // Calculate potential  Abort
  int startIndex = toIndex(start_x, start_y);
  int endIndex = toIndex(end_x, end_y);
  std::fill(potential, potential + ns_, POT_HIGH);

  Index current(startIndex, hx_->CalcHeuristic(start_x, start_y, start_x, start_y, end_x, end_y, (float)neutral_cost_),
                0);
  potential[current.i] = 0;

  // clear the queue
  clearpq(queue_);

  queue_.push(current);

  while (!queue_.empty() && current.i != endIndex && cycle < cycles)
  {
    if (queue_.empty())
      return false;

    current = queue_.top();
    queue_.pop();

    calcPotentialAndAddCandidate(costs, potential, current, current.i + 1, end_x, end_y, start_x, start_y, false);
    calcPotentialAndAddCandidate(costs, potential, current, current.i - 1, end_x, end_y, start_x, start_y, false);
    calcPotentialAndAddCandidate(costs, potential, current, current.i + nx_, end_x, end_y, start_x, start_y, false);
    calcPotentialAndAddCandidate(costs, potential, current, current.i - nx_, end_x, end_y, start_x, start_y, false);

    if (hx_->getExpansionSize() == expansion_8)
    {
      calcPotentialAndAddCandidate(costs, potential, current, current.i + nx_ + 1, end_x, end_y, start_x, start_y,
                                   true);
      calcPotentialAndAddCandidate(costs, potential, current, current.i + nx_ - 1, end_x, end_y, start_x, start_y,
                                   true);
      calcPotentialAndAddCandidate(costs, potential, current, current.i - nx_ + 1, end_x, end_y, start_x, start_y,
                                   true);
      calcPotentialAndAddCandidate(costs, potential, current, current.i - nx_ - 1, end_x, end_y, start_x, start_y,
                                   true);
    }

    cycle++;
  }

  if (current.i == endIndex)
  {
    ROS_INFO("POTENTIAL MAP FOUND %i", cycle);
    return true;
  }

  return false;
}

/**ll
Private helper
**/
void AStarTuwExpansion::calcPotentialAndAddCandidate(unsigned char *costs, float *potential, Index lastNode, int index,
                                                     int end_x, int end_y, int start_x, int start_y, bool diagonal)
{
  float potentialPrev = potential[lastNode.i];
  // Check Boundries
  if (index < 0 || index >= ns_)
    return;

  // Dont update allready found potentials
  if (potential[index] < POT_HIGH)
    return;

  if (costs[index] >= lethal_cost_ && !(unknown_ && costs[index] == costmap_2d::NO_INFORMATION))
    return;

  int xVal = index % nx_;
  int yVal = index / nx_;

  float dist_to_start = sqrt((start_x - xVal) * (start_x - xVal) + (start_y - yVal) * (start_y - yVal)) * neutral_cost_;

  float pot = potentialPrev + costs[index] +
              hx_->CalcCosts(xVal, yVal, start_x, start_y, end_x, end_y, lastNode.dist, neutral_cost_, diagonal);
  // float pot = p_calc_->calculatePotential(potential, costs[index] + hx_->CalcCosts(xVal, yVal, start_x, start_y,
  // end_x, end_y, lastNode.dist, neutral_cost_, diagonal), index, potentialPrev);
  potential[index] = pot;

  float h = hx_->CalcHeuristic(xVal, yVal, start_x, start_y, end_x, end_y, neutral_cost_);

  queue_.push(Index(index, potential[index] + h, dist_to_start));
}
} // namespace global_planner
