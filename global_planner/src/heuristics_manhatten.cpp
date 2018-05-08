#include <global_planner/heuristics_manhatten.h>

namespace global_planner
{
float HeuristicsManhatten::CalcHeuristic(double act_x, double act_y, double start_x, double start_y, double end_x,
                                         double end_y, float neutral_cost)
{
  //     return ((abs(end_x - act_x) + abs(end_y - act_y)) * neutral_cost) * heuristics_multiplier_;
}

float HeuristicsManhatten::CalcCosts(double act_x, double act_y, double start_x, double start_y, double end_x,
                                     double end_y, float lastDistToStart, float neutral_cost, bool diagonal)
{
  return neutral_cost;
}
}
