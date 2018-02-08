#include <global_planner/heuristics_diagonal.h>
#include <math.h>
#include <algorithm>

namespace global_planner {
  
  float HeuristicsDiagonal::CalcHeuristic(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float neutral_cost)
  {
    float dx = abs(act_x-end_x);
    float dy = abs(act_y-end_y);
    
    
    float dx1 = act_x - end_x;
    float dy1 = act_y - end_y;
    
    float dx2 = start_x - end_x;
    float dy2 = start_y - end_y;
    //Tilebreaker to move diagonal
    float cross = abs(dx1*dy2 - dx2*dy1);
    
    float minVal = dx<dy? dx:dy;
    
    float D = neutral_cost;
    float D2 = neutral_cost * sqrt(2);
    return (D * (dx + dy) + (D2 - 2 * D) * minVal + cross * 0.001)  * heuristics_multiplier_;
  }
  
  float HeuristicsDiagonal::CalcCosts(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float lastDistToStart, float neutral_cost, bool diagonal)
  {
    if(diagonal)
      return neutral_cost * sqrt(2);
    else
      return neutral_cost;
  }
  
  expansion_type HeuristicsDiagonal::getExpansionSize()
  {
    return expansion_8;
  };

}
