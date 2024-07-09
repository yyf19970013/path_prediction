#ifndef ASTAR_H
#define ASTAR_H

#include "Common.h"
#include "ConstraintTable.h"
#include "AstarNode.h"
#include "OutIn.h"

class Astar{

public:
    Astar() = default;
    ~Astar() = default;
    inline int compute_h_value(const coordinate& start, const coordinate& goal){return abs(goal.x - start.x) + abs(goal.y - start.y);}
    // int get_w_val(const Map& map, const coordinate& next);

    Path run(const Map& map, const coordinate& start, const coordinate& goal);
    Path run(const Map& map, const coordinate& start, const coordinate& goal, ConstraintTable& rt, int agvClass);

    Path updatePath(const AstarNode* node);
    sidePath updateSidePath(const Path& path, const vector<int>& conner_index);

    void releaseAllNodes();
    list<coordinate> getNeighbors(const Map& map, const AstarNode* node);
    void setMapsize(Map map);
    void drawPath(const Path& p, const vector<vector<int>>& m);


    double min_f_val;
    double path_cost;



    fibonacci_heap< AstarNode*, compare<AstarNode::compare_node> > open_list;
	fibonacci_heap< AstarNode*, compare<AstarNode::secondary_compare_node> > focal_list;
	unordered_set< AstarNode*, AstarNode::Hasher, AstarNode::EqNode> allNodes_table;

    int row_;
    int col_;
    Common common_;
    OutIn oi;
private:

};

#endif