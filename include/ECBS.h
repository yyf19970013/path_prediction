#include "Common.h"
#include "ECBSNode.h"
#include "ReservationTable.h"
#include "Path.h"
#include "Astar.h"

class ECBS{

public:
    ECBS();
    ~ECBS() = default;
    bool run(const vector<tuple<int,int,int,int>>& starts,
             const vector<tuple<int,int,int,int>>& ends,
             int time_limits);
    bool generate_root_node();

    ECBSNode* pop_node();
    void push_node(ECBSNode* node);

    void update_paths(ECBSNode* node);
    int get_w(ECBSNode* node);
    void choose_conflict(ECBSNode& node);
    void resolve_conflict(shared_ptr<Conflict> conflict, ECBSNode* n1, ECBSNode* n2);
    bool generate_child(ECBSNode* node, ECBSNode* parent);

    void find_conflict(int len, const Path& p1, const Path& p2, bool a_v, bool b_v);
    void find_confilcts(int len, ECBSNode* node);

    void get_solution();

private:

    typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::compare_node> > heap_open_t;
    typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::secondary_compare_node> > heap_focal_t;
    heap_open_t open_list_;
    heap_focal_t focal_list_;

    unordered_map<int,map> maps_;

    Astar Astar_;
    ReservationTable rt_;

    ECBSNode* best_node_;
    ECBSNode* dummy_start_;

    bool solution_found_ = false;
    vector<tuple<int,int,int,int>> starts_;
    vector<tuple<int,int,int,int>> goals_;
    int num_of_agents_;

    vector<Path*> best_paths_;
    
    OutIn oi;
};