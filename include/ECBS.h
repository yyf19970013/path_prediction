#ifndef ECBS_H
#define ECBS_H

#include "Common.h"
#include "ECBSNode.h"
#include "Astar.h"
#include "OutIn.h"

class ECBS{

public:

    ECBS():best_node_(nullptr), dummy_start_(nullptr){}
    ~ECBS() = default;
    void initialize(vector<Map>& maps, vector<pair<coordinate, int>>& starts, vector<coordinate>& ends, int min_len, double turnstime);
    bool run();
    bool generate_root_node();
    void set_min_len(int len){min_len_ = len;}//最小Len接口

    // ECBSNode* pop_node();
    void push_node(ECBSNode* node);

    void update_paths(ECBSNode* node);

    void choose_conflict(ECBSNode& node);
    
    void resolve_sideConflict(ECBSNode* n1, ECBSNode* n2);
    bool generate_child(ECBSNode* node);

    void find_sideConflicts(int len, ECBSNode* node);
    void find_sideConflict(const Path& iLine, const Path& jLine, //TODO：做个开关来调控是否要计算点碰撞信息
                            int i_ori, int j_ori,
                            int iID, int jID,
                            bool x_driect, int move_driect,
                            ECBSNode* node);

    void find_vertexConflicts(int len, ECBSNode* node);
    void find_vertexConflict(const Path& iLine, const Path& jLine,
                             int i_ori, int j_ori,
                             int iID, int jID,
                             ECBSNode* node);

    coordinate getIntersectionPoint(const Path& line1, const Path& line2);
    vector<int> getIntersectionLine(const Path& line1, const Path& line2, bool isX);
    bool isInLine(const Path& path, const coordinate& coord);

    Path minTurnsPath(const Path& path, vector<size_t> conner_idx, int agvClass);
    bool isCrossObs(const Path& path, int ID);
    Path connectPath(const coordinate& start, const coordinate& end);
    Path getPointPath(const vector<coordinate>& turns);
    bool find_path(ECBSNode* node);
    void resolveVertexConf(ECBSNode* node, vector<bool> updated);
    bool validCoord(const coordinate& coord, const int& ID);

    void getConneridx(const Path& path, vector<size_t>& coidx);

    vector<Path> get_runpaths();
    vector<Path> runPaths;



    typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::compare_node> > heap_open_t;
    typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::secondary_compare_node> > heap_focal_t;
    heap_open_t open_list_;
    heap_focal_t focal_list_;

    vector<Map> maps_;
    int col_;//y
    int row_;//x

    Astar Astar_;
    ConstraintTable ct_;

    ECBSNode* best_node_;
    ECBSNode* dummy_start_;

    bool solution_found_ = false;
    vector<pair<coordinate, int>> starts_;
    vector<coordinate> ends_;
    int num_of_agents_;

    vector<Path> paths_;
    vector<sidePath> spaths_;
    vector<Path> runPaths_;

    int randomNum_;
    double turntime_;
    vector<vertexConflict> vertexConflicts;
    
    OutIn oi;
    random_device rd;

    Common common;
    int min_len_;

private:

};

#endif