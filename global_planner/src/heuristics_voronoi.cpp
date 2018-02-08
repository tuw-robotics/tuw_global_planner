#include <global_planner/heuristics_voronoi.h>
#include <math.h>
#include <algorithm>


namespace global_planner {
  
  float HeuristicsVoronoi::CalcHeuristic(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float neutral_cost)
  {
    return (sqrt((end_x - act_x)*(end_x - act_x) + (end_y - act_y)*(end_y - act_y)) * neutral_cost)  * heuristics_multiplier_;
  }
  
  float HeuristicsVoronoi::CalcCosts(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float lastDistToStart, float neutral_cost, bool diagonal)
  {
    return 1;
  }
  expansion_type HeuristicsVoronoi::getExpansionSize()
  {
    return expansion_4;
  };

}
