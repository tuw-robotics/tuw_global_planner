#include <global_planner/heuristics_euclidean.h>
#include <math.h>
#include <algorithm>


namespace global_planner {
  
  float HeuristicsEuclidean::CalcHeuristic(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float neutral_cost)
  {
    return (sqrt((end_x - act_x)*(end_x - act_x) + (end_y - act_y)*(end_y - act_y)) * neutral_cost)  * heuristics_multiplier_;
  }
  
  float HeuristicsEuclidean::CalcCosts(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float lastDistToStart, float neutral_cost, bool diagonal)
  {
    float dist_to_start = sqrt((start_x - act_x)*(start_x - act_x) + (start_y - act_y)*(start_y - act_y)) * neutral_cost;
    //Take care of zero or negative edge Weights!!!!
    float weight = (dist_to_start - lastDistToStart);
    return weight > 0.001 ? weight : 0.001;
  }
  expansion_type HeuristicsEuclidean::getExpansionSize()
  {
    return expansion_4;
  };

}
