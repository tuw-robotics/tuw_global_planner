
#ifndef _HEURISTICS_H
#define _HEURISTICS_H

namespace global_planner
{
typedef enum expansion_t
{
  expansion_4,
  expansion_8
} expansion_type;

class Heuristics
{
public:
  /**
 * @brief Construct a new Heuristics object
 * 
 * @param heuristics_multiplier the multiplier used to weight the heuristic (e.g. 0 = no heuristic/dirjkstra)
 */
  Heuristics(float heuristics_multiplier)
  {
    heuristics_multiplier_ = (heuristics_multiplier < 1) ? heuristics_multiplier : 1;
  };
  /**
 * @brief calculates the heuristic
 * 
 * @param neutral_cost the costs for a step to the next point
 * @return float the heuristics result
 */
  virtual float CalcHeuristic(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float neutral_cost)
  {
    return 0; // Dijkstra expansion
  };
  /**
   * @brief 
   * 
   * @param lastDistToStart marker to remember the last distance to start calculated
   * @param neutral_cost the costs to expand from one pixel to another
   * @param diagonal marks if the expension is diagonal
   * @return the costs to expand
   */
  virtual float CalcCosts(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float lastDistToStart, float neutral_cost, bool diagonal)
  {
    return neutral_cost;
  };
  /**
   * @brief Get the Expansion Size object (this expension type defines the number of neighbors the expander is allowed to expand to (4 or 8))
   * 
   * @return expansion_type 
   */
  virtual expansion_type getExpansionSize()
  {
    return expansion_4;
  };

protected:
  float heuristics_multiplier_;
};

} // end namespace global_planner
#endif
