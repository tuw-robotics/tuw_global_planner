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
#include <global_planner/voronoi_map.h>
#include <costmap_2d/cost_values.h>
#include <global_planner/heuristics.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace global_planner {

#define POT_HIGH	1.0e10
  
VoronoiExpansion::VoronoiExpansion(PotentialCalculator* p_calc, int xs, int ys, Heuristics *hx, voronoi_map::VoronoiMapProvider *v_map) :
        Expander(p_calc, xs, ys) {
	  hx_ = hx;
	  v_map_ = v_map;
	  
  distance_field_.reset(new float[ns_]);	
  voronoi_graph_.reset(new int8_t[ns_]);
  global_map_.reset(new int8_t[ns_]);
}

void VoronoiExpansion::getMaps(float *distance_field, int8_t *voronoi_graph, int8_t *global_map, grid_map::GridMap *voronoi_map)
{
  auto& distfield = voronoi_map->get("distfield");
  auto& voronoi = voronoi_map->get("voronoi");
  auto& map = voronoi_map->get("map");
  
  for (grid_map::GridMapIterator iterator(*voronoi_map); !iterator.isPastEnd(); ++iterator) 
  {
    const grid_map::Index mapIndex = iterator.getUnwrappedIndex();
    int arrayIndex = (ny_-1-mapIndex[1])*nx_+(nx_-1-mapIndex[0]);	//Maybe use Opencv to get rid of these calculation (turn 90deg and mirror)
    distance_field[arrayIndex] = distfield(iterator.getLinearIndex());	
    voronoi_graph[arrayIndex] = voronoi(iterator.getLinearIndex());
    global_map[arrayIndex] = map(iterator.getLinearIndex());
  }
}


bool VoronoiExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential)
{
  if(!v_map_->map_available())
  {
    ROS_ERROR("NO VORONOI MAP RECEIVED");
    return false;
  }
  
  
  std::shared_ptr<grid_map::GridMap> v_map_ptr = v_map_->get_grid_map();
  grid_map::GridMap *voronoi_map = v_map_ptr.get();
  
  float *distance_field = distance_field_.get();
  int8_t *voronoi_graph = voronoi_graph_.get();
  int8_t *global_map = global_map_.get();
  
  getMaps(distance_field, voronoi_graph, global_map, voronoi_map);
  
  Index startPoint = Index(start_x,start_y,nx_,0,0,0);
  Index endPoint = Index(end_x,end_y,nx_,0,0,0);
  
  
  
  
  
  
  //EndPoint candidates //Dikjstra instead of going along the distance_field because sometimes the voronoi map is a little bit of the distance field (if so gradient mehtod would fail)
  std::fill(potential, potential + ns_, POT_HIGH);
  Index endPointGraph = expandDijkstraToVoronoi(endPoint, startPoint, cycles, global_map, voronoi_graph, distance_field, potential);
  if(endPointGraph.i < 0)
    return false;
  int endPointImprovementSteps = (int)distance_field[endPointGraph.i];
  endpoints_ = findVoronoiCandidates(endPointGraph, global_map, voronoi_graph, distance_field, potential, endPointImprovementSteps);
  
  //StartPoint candidates //Reset map for Start calculation (in case Start is next to end)
  std::fill(potential, potential + ns_, POT_HIGH);
  Index startPointGraph = expandDijkstraToVoronoi(startPoint,endPoint, cycles, global_map, voronoi_graph, distance_field, potential);
  if(startPointGraph.i < 0)
    return false;
  int startPointImprovementSteps = (int)distance_field[startPointGraph.i];
  std::list<Index> startpoints = findVoronoiCandidates(startPointGraph, global_map, voronoi_graph, distance_field, potential, startPointImprovementSteps);
  startpoints.push_back(startPointGraph);
  
  
  
  
  
  
  //Start the Algorigthm by expanding on the Graph
  std::fill(potential, potential + ns_, POT_HIGH);  											//For having a high startpotential 
  potential[startPointGraph.i] = startPointGraph.potential;
  Index endPointGraphReal = expandVoronoi(startPointGraph, endPointGraph, cycles, global_map, voronoi_graph, distance_field, potential);
  if(endPointGraphReal.i < 0)
    return false;
  
  
  //Draw lines to all startpoints and the endpoint (there is allways a straight line between voronoi Graph and endPoint)
  expandLine(endPointGraphReal, endPoint, endPointGraphReal.potential, -1, potential);
  for (std::list<Index>::iterator it=startpoints.begin(); it != startpoints.end(); ++it)
  {
    expandLine(startPoint, *it, 0, startPointGraph.potential, potential);
  }
  
  return true;
}

void VoronoiExpansion::expandLine(Index start, Index end, float startPotential, float endPotential, float* potential)
{
  int start_x = start.i%nx_;
  int start_y = start.i/nx_;
  int end_x = end.i%nx_;
  int end_y = end.i/nx_;
  
  int steps = std::max(abs(start_x - end_x),abs(start_y - end_y));
  
  float neutralCosts = 1;
  if(endPotential >= 0)
    (endPotential - startPotential) / steps;
  
  float improvementX = (float)(end_x - start_x) / (float)steps;
  float improvementY = (float)(end_y - start_y) / (float)steps;
  potential[start.i] = startPotential;
  float actPotential = startPotential;
    
  for(int i = 0; i < steps; i++)
  {
    actPotential += neutralCosts;
    potential[start.offsetDist(improvementX * i,improvementY * i,nx_,ny_).i] = actPotential;
  }
}

std::list<VoronoiExpansion::Index> VoronoiExpansion::findVoronoiCandidates(Index start, int8_t *map, int8_t* voronoi_map, float *dist_field, float *potential, int steps)
{
  int cycle = 0;  
  std::list<Index> candidates;
  
  candidates.push_back(start);

  Index current(start.i, potential[start.i],0, potential[start.i]);
  potential[current.i] = 0;
  current.potential = 0;

  //clear the queue
  clearpq(queue_);
  queue_.push(current);

  while(!queue_.empty())
  {
    if(queue_.empty())
      return candidates;

    current = queue_.top();
    queue_.pop();
    
    //ROS_INFO("c %i %i %i", cycle, (int)current.potential, steps);
    //If we have found a max potential return the pixel
    if(current.potential >= steps)
    {
      //ROS_INFO("Padd %i", current.i);
      candidates.push_back(current);
      
    }
    else if(cycle < steps * 10)	//Maximum ten new endpoints...
    {
      addExpansionCandidate(start, current, current.offsetDist(1,0,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
      addExpansionCandidate(start, current, current.offsetDist(0,1,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
      addExpansionCandidate(start, current, current.offsetDist(-1,0,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
      addExpansionCandidate(start, current, current.offsetDist(0,-1,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
      addExpansionCandidate(start, current, current.offsetDist(1,1,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
      addExpansionCandidate(start, current, current.offsetDist(-1,1,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
      addExpansionCandidate(start, current, current.offsetDist(-1,-1,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
      addExpansionCandidate(start, current, current.offsetDist(1,-1,nx_,ny_), start, map, voronoi_map, dist_field, potential, true, true);
    }

    cycle++;
  }
  
  return candidates;
}
  


VoronoiExpansion::Index VoronoiExpansion::expandDijkstraToVoronoi(Index start, Index end, int cycles, int8_t *map, int8_t *voronoi_map, float *dist_field, float* potential)
{  
  int cycle = 0;   
  
  Index current(start.i, 0,0,0);
  potential[current.i] = 0;

  //clear the queue
  clearpq(queue_);
  queue_.push(current);

  while(!queue_.empty() && voronoi_map[current.i] >= 0 && cycle < cycles)
  {
    if(queue_.empty())
      return Index(-1,0,0,0);

    current = queue_.top();
    queue_.pop();
    
    //If we have found a max potential return the pixel
    if(voronoi_map[current.i] < 0)
    {
      potential[current.i] = current.potential;
      return current;
    }


    addExpansionCandidate(start, current, current.offsetDist(1,0,nx_,ny_), end, map, voronoi_map, dist_field, potential, false, true);
    addExpansionCandidate(start, current, current.offsetDist(0,1,nx_,ny_), end, map, voronoi_map, dist_field, potential, false, true);
    addExpansionCandidate(start, current, current.offsetDist(-1,0,nx_,ny_), end, map, voronoi_map, dist_field, potential, false, true);
    addExpansionCandidate(start, current, current.offsetDist(0,-1,nx_,ny_), end, map, voronoi_map, dist_field, potential, false, true);

    cycle++;
  }
  
  if(cycle >= cycles)
    return Index(-1,-1,-1,-1);
  else
    return start;
}


VoronoiExpansion::Index VoronoiExpansion::expandToEnd(Index start, Index end, int cycles, int8_t *map, int8_t *voronoi_map, float* distance_map, float* potential)
{
  int cycle = 0;   

  Index current(start.i, potential[start.i],0, potential[start.i]);

  //clear the queue
  clearpq(queue_);
  queue_.push(current);

  while(!queue_.empty() && current.i != end.i && cycle < cycles)
  {
    if(queue_.empty())
      return Index(-1,0,0,0);

    current = queue_.top();
    queue_.pop();
    
    //If we have found a max potential return the pixel
    if(current.i == end.i)
    {
      potential[current.i] = current.potential;
      return current;
    }


    addExpansionCandidate(start, current, current.offsetDist(1,0,nx_,ny_), end, map, voronoi_map, distance_map, potential, false, false);
    addExpansionCandidate(start, current, current.offsetDist(0,1,nx_,ny_), end, map, voronoi_map, distance_map, potential, false, false);
    addExpansionCandidate(start, current, current.offsetDist(-1,0,nx_,ny_), end, map, voronoi_map, distance_map, potential, false, false);
    addExpansionCandidate(start, current, current.offsetDist(0,-1,nx_,ny_), end, map, voronoi_map, distance_map, potential, false, false);

    cycle++;
  }
  
  if(cycle >= cycles)
    return Index(-1,-1,-1,-1);
  else
    return start;

}

void VoronoiExpansion::addExpansionCandidate(Index start, Index current, Index next, Index end, int8_t *map, int8_t *voronoi_map, float *dist_field, float* potential, bool expand_on_voronoi, bool expand_dijkstra_to_voronoi)
{
  float potentialPrev = potential[current.i];
  //Check Boundries
  if(next.i < 0 || next.i > ns_)
      return;

  //Dont update allready found potentials
  if (potential[next.i] < POT_HIGH)
      return;

  if(map[next.i] > 0)
    return;
  
  if(expand_on_voronoi)
  {
    if(voronoi_map[next.i] >= 0)		//If not on voronoi graph return
      return;
  }
    
  float pot = 0;
  float dist = 0;
  float weight = 0;
  if(expand_dijkstra_to_voronoi)
  {
    pot = potentialPrev + neutral_cost_;
    weight = -dist_field[next.i];		//Set wait to prefer higher potentials
  }
  else
  {
    int start_x = start.i%nx_;
    int start_y = start.i/ny_;
    int end_x = end.i%nx_;
    int end_y = end.i/ny_;
    int next_x = next.i % nx_;
    int next_y = next.i /nx_;
    
    dist = sqrt((start_x - next_x)*(start_x - next_x) + (start_y - next_y)*(start_y - next_y)) * neutral_cost_; 
    pot = potentialPrev + hx_->CalcCosts(next_x, next_y, start_x, start_y, end_x, end_y, current.dist, neutral_cost_, false);
    float h = hx_->CalcHeuristic(next_x, next_y, start_x, start_y, end_x, end_y, neutral_cost_);
    weight = pot + h;
  }
  
  if(expand_dijkstra_to_voronoi && expand_on_voronoi)
  {
    potential[next.i] = pot;
  }
  else if(expand_dijkstra_to_voronoi)
  {
    if(voronoi_map[next.i] >= 0)
      potential[next.i] = pot;
    else
    {
      weight = - dist_field[next.i]; //to move up pixel in queue
    }
  }
  else
  {
    if(next.i != end.i)
      potential[next.i] = pot;
    else
    {
      weight = 0; //to move up pixel in queue
    }
  }
    
  queue_.push(Index(next.i, weight, dist, pot));

}

VoronoiExpansion::Index VoronoiExpansion::expandVoronoi(Index start, Index end, int cycles, int8_t *map, int8_t *voronoi_map, float* distance_map, float* potential)
{
  int cycle = 0; 

  Index current(start.i, potential[start.i],0, potential[start.i]);

  //clear the queue
  clearpq(queue_);
  queue_.push(current);

  
  while(!queue_.empty()  && cycle < cycles && !isEndpoint(current)) //&& !isInGoalZone(current.i)
  {
    if(queue_.empty())
      return Index(-1,0,0,0);

    current = queue_.top();
    queue_.pop();
    
    //If we have found a max potential return the pixel
    if(isEndpoint(current))
    {
      potential[current.i] = current.potential;
      return current;
    }
    

    addExpansionCandidate(start, current, current.offsetDist(1,0,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);
    addExpansionCandidate(start, current, current.offsetDist(0,1,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);
    addExpansionCandidate(start, current, current.offsetDist(-1,0,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);
    addExpansionCandidate(start, current, current.offsetDist(0,-1,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);

    addExpansionCandidate(start, current, current.offsetDist(1,1,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);
    addExpansionCandidate(start, current, current.offsetDist(-1,1,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);
    addExpansionCandidate(start, current, current.offsetDist(-1,-1,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);
    addExpansionCandidate(start, current, current.offsetDist(1,-1,nx_,ny_), end, map, voronoi_map, distance_map, potential, true, false);
    cycle++;
  }
  
  if(cycle >= cycles)
    return Index(-1,-1,-1,-1);
  else
    return end;

}


bool VoronoiExpansion::isEndpoint(VoronoiExpansion::Index p)
{
  for (std::list<Index>::iterator it=endpoints_.begin(); it != endpoints_.end(); ++it)
  {
    if(p.i == it->i)
      return true;
  }
  
  return false;
}

}
