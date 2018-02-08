
#ifndef _HEURISTICS_H
#define _HEURISTICS_H


namespace global_planner {

typedef enum expansion_t
{
  expansion_4,
  expansion_8
}expansion_type;

class Heuristics
{
    public:
        Heuristics(float heuristics_multiplier)
	{
	  heuristics_multiplier_ = (heuristics_multiplier < 1) ? heuristics_multiplier : 1;
	};
        virtual float CalcHeuristic(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float neutral_cost)
	{
	  return 0; //Dijkstra expansion
	};
	virtual float CalcCosts(double act_x, double act_y, double start_x, double start_y, double end_x, double end_y, float lastDistToStart, float neutral_cost, bool diagonal)
	{
	  return neutral_cost;
	};
	virtual expansion_type getExpansionSize()
	{
	  return expansion_4;
	};
  protected:
	float heuristics_multiplier_;

};


} //end namespace global_planner
#endif
