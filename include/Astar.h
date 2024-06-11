#include "Common.h"
#include "ReservationTable.h"
#include "AstarNode.h"

class Astar{

public:
    Astar();
    ~Astar();
    inline int compute_h_value(const coordinate& start, const coordinate& goal){return (goal.first - start.first) + (goal.second - start.first);}
    int get_w_val(const map& map, const coordinate& next);

    Path run(const map& map, const coordinate& start, const coordinate& goal);
    Path run(const map& map, const coordinate& start, const coordinate& goal, ReservationTable& rt);

    Path updatePath(const AstarNode* node);
    sidePath updateSidePath(const Path& path);

    void releaseAllNodes();
    list<coordinate> getNeighbors(const map& map, const AstarNode* node);
    

    double min_f_val;
    double path_cost;

private:

    fibonacci_heap< AstarNode*, compare<AstarNode::compare_node> > open_list;
	fibonacci_heap< AstarNode*, compare<AstarNode::secondary_compare_node> > focal_list;
	unordered_set< AstarNode*, AstarNode::Hasher, AstarNode::EqNode> allNodes_table;

    Common common_;

};