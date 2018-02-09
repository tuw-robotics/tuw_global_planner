#ifndef VORONOI_MAP_H
#define VORONOI_MAP_H

#include <global_planner/expander.h>
#include <global_planner/heuristics.h>
#include <algorithm>
#include <memory>
#include <queue>
#include <global_planner/voronoi_path_generator.h>
#include <opencv/cv.h>


namespace global_planner
{

    class VoronoiExpansion : public Expander, public VoronoiPathGenerator
    {
        private:
            std::unique_ptr<float[]> distance_field_;
            std::unique_ptr<int8_t[]> voronoi_graph_;
            std::unique_ptr<int8_t[]> global_map_;
            template <class T, class S, class C>
            void clearpq ( std::priority_queue<T, S, C> &q )
            {
                q = std::priority_queue<T, S, C>();
            }
            class Index
            {
                public:
                    Index ( int index, float c, float d, float p )
                    {
                        i = index;
                        weight = c;
                        dist = d;
                        potential = p;
                    }
                    Index ( int x_val, int y_val, int nx, float c, float d, float p )
                    {
                        i = x_val + y_val * nx;
                        weight = c;
                        dist = d;
                        potential = p;
                    }
                    Index offsetDist ( int dist_x, int dist_y, int nx, int ny )
                    {
                        int x_val = ( i % nx ) + dist_x;
                        int y_val = ( i / nx ) + dist_y;

                        if ( x_val < 0 || x_val > nx || y_val < 0 || y_val > ny )
                            return Index ( -1, -1, -1, -1 );

                        return Index ( x_val, y_val, nx, 0, 0, 0 );
                    }
                    int i;
                    float weight;
                    float dist;
                    float cost;
                    float potential;
            };

            struct greater1
            {
                bool operator() ( const Index &a, const Index &b ) const
                {
                    return a.weight > b.weight;
                }
            };
            std::priority_queue<Index, std::vector<Index>, greater1> queue_;
            Heuristics *hx_;
            int goalzone_x_min_;
            int goalzone_x_max_;
            int goalzone_y_min_;
            int goalzone_y_max_;
            std::list<Index> endpoints_;
            bool gotMap_;
            Eigen::Vector2d origin_;
            float resolution_;
            cv::Mat map_;
            cv::Mat voronoi_;
            cv::Mat distfield_;
            size_t currentHash_;

            /**
            * @brief adds a expansion candidate to the expansion queue (sorted by weight)
            * @param start              The start point (of the graph) (only important for the heuristic and the potential calculation)
            * @param current            The previos point from which we expandDijkstraToEnd
            * @param next               The point we want to expand to
            * @param end                The final end point (of the graph) (only important for the heuristic and the potential calculation)
            * @param potential          the calculatet Potential map for return
            * @param map                the used map showing static obstacles
            * @param voronoi_map            the voronoi map of the problem (GVD)
            * @param dist_field         the distance transform of the map
            * @param expand_on_voronoi      if true only expands on the voronoi graph
            * @param expand_dijkstra_to_voronoi expand to the end point or to a voronoi point (used to abort the expansion)
            **/
            void addExpansionCandidate ( Index start, Index current, Index next, Index end, int8_t *map, int8_t *voronoi_map, float *dist_field, float *potential, bool expand_on_voronoi, bool expand_dijkstra_to_voronoi );

            /**
            * @brief expands a graph with dijkstra until a voronoi pixel is found
            * @param start      The start point
            * @param goal       The final goal (is only important for the potential calculation (minor impact)
            * @param cycles     maximum of iterations
            * @param potential  the calculatet Potential map for return
            * @param map        the used map showing static obstacles
            * @param voronoi_map    the voronoi map of the problem (GVD)
            * @param dist_field the distance transform of the map
            * @return               the index of the voronoi Node (-1 for nothing found),
            **/
            Index expandDijkstraToVoronoi ( Index start, Index end, int cycles, int8_t *map, int8_t *voronoi_map, float *dist_field, float *potential );

            /**
            * @brief expands a graph with dijkstra until the endpoint is found
            * @param start      The start point
            * @param goal       The final goal
            * @param cycles     maximum of iterations
            * @param map        the used map showing static obstacles
            * @param voronoi_map    the voronoi map of the problem (GVD)
            * @param dist_field the distance transform of the map
            * @param potential  the calculatet Potential map for return
            * @return               the index of the voronoi Node (-1 for nothing found),
            **/
            Index expandToEnd ( Index start, Index end, int cycles, int8_t *map, int8_t *voronoi_map, float *dist_field, float *potential );

            /**
            * @brief expands a graph with dijkstra only on the voronoi graph until the goal pixel is found
            * @param start      The start point (on the voronoi graph)
            * @param end        The final goal (on the voronoi graph)
            * @param cycles     maximum of iterations
            * @param potential  the calculatet Potential map for return
            * @param map        the used map showing static obstacles
            * @param voronoi_map    the voronoi map of the problem (GVD)
            * @param dist_field the distance transform of the map
            * @return               the index of the voronoi Node (-1 for nothing found),
            **/
            Index expandVoronoi ( Index start, Index end, int cycles, int8_t *map, int8_t *voronoi_map, float *dist_field, float *potential );

            /**
            * @brief translates along the voronoi path for steps to get multiple start or end positions for a shorter path
            * @param start      the point to start the expansion in all directions
            * @param map        the used map showing static obstacles
            * @param voronoi_map    the voronoi map of the problem (GVD)
            * @param dist_field the distance transform of the map
            * @param potential  the calculatet Potential map for return
            * @param steps      the number of steps to go along the path
            * @return       a list of start/end point candidates on the voronoi path
            **/
            std::list<Index> findVoronoiCandidates ( Index start, int8_t *map, int8_t *voronoi_map, float *dist_field, float *potential, int steps );

            /**
            * @brief checks  if p lies on one of the endpoints defined in the list endpoints_ (has to be initialized manually)
            * @param p  the point to check
            * @return   true if p lies on any of the enpoints
            **/
            bool isEndpoint ( Index p );
            /**
             * @brief   generates the 3 different maps out of the voronoi_map
             * @param distance_field    the array of size ns_ where the distance_field is written to
             * @param global_map        the array of size ns_ with ns_ where the global_map is written to
             * @param voronoi_map       the array of size ns_ with ns_ where the voronoi_map is written to
             **/
            void getMaps ( float *distance_field, int8_t *voronoi_graph, int8_t *global_map );

            /**
             * @brief           draws a line on the potential map with increasing (or decreasing) potential (if endPotential is smaller 0 the stepCosts are 1 else it is (endPot-startPot)/steps
             * @param start         the startpoint of the line
             * @param end           the end point of the line
             * @param startPotential    the potential at the start point
             * @param endPotential      the potential of the endPoint
             * @param potential     the calculatet Potential map for return
             **/
            void expandLine ( Index start, Index end, float startPotential, float endPotential, float *potential );
            /**
             * @brief               calculates the hash of a map to reidentify maps
             * @param _map          the map to identify As CV_8UC1 or CV_8SC1
             * @param _origin       the origin of the map
             * @param _resolution   the resolution of the map 
             */
            std::size_t getHash(const cv::Mat &_map, Eigen::Vector2d _origin, float _resolution);
        public:
            /**
            * @brief the constructor
            * @param p_calc the calculation method for the potential
            * @param xs x size of the tuw_voronoi_map
            * @param ys y size of the tuw_voronoi_map
            * @param hx the used heuristics
            **/
            VoronoiExpansion ( PotentialCalculator *p_calc, int xs, int ys, Heuristics *hx );

            /**
            * @brief calculates the Potentials to the goal position and returns a potential Map
            * @param costs        Costs from paoint to Point (0 means empty, costmap_2d::COST_LETHAL means obstacle costmap_2d::COST_INSCRIBED is the blured map -1 is no Information)
            * @param start_x s    tartx
            * @param start_y      starty
            * @param goal_x       goalx
            * @param goal_y       goalx
            * @param cycles       maximum of iterations
            * @param potential    the calculatet Potential map for return
            * @param voronoi_map    the voronoi map of the problem (containing GVD and potentials)
            * @return            success or not
            **/
            bool calculatePotentials ( unsigned char *costs, double start_x, double start_y, double end_x, double end_y, int cycles, float *potential );

            
            /**
             * @brief saves a newly received maps
             * @param _map      the map to save and compute 
             */
            void setNewMap(cv::Mat _map, Eigen::Vector2d _origin, float _resoltuion );
    };
} //end namespace global_planner





#endif
