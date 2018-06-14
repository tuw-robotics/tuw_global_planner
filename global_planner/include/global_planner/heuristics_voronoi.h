
#ifndef _HEURISTICS_VORONOI_H
#define _HEURISTICS_VORONOI_H

#include <global_planner/heuristics.h>
#include <vector>
#include <algorithm>

namespace global_planner
{
class HeuristicsVoronoi : public Heuristics
{
public:
  /**
 * @brief Construct a new Heuristics Voronoi object
 * 
 * @param heuristics_multiplier 
 */
  HeuristicsVoronoi(float heuristics_multiplier) : Heuristics(heuristics_multiplier)
  {
  }
  /**
 * @brief calculates the heuristic
 * 
 * @param neutral_cost the costs for a step to the next point
 * @return float the heuristics result
 */
  virtual float CalcHeuristic(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float neutral_cost);
  /**
   * @brief 
   * 
   * @param lastDistToStart marker to remember the last distance to start calculated
   * @param neutral_cost the costs to expand from one pixel to another
   * @param diagonal marks if the expension is diagonal
   * @return the costs to expand
   */
  virtual float CalcCosts(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float lastDistToStart, float neutral_cost, bool diagonal);
  /**
   * @brief Get the Expansion Size object (this expension type defines the number of neighbors the expander is allowed to expand to (4 or 8))
   * 
   * @return expansion_type 
   */
  virtual expansion_type getExpansionSize();
};

} // end namespace global_planner
#endif
