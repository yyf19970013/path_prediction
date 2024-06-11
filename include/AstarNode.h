#include "Common.h"

class AstarNode
{
public:
    coordinate xy;
    double g_val;
    double h_val;
    AstarNode* parent;
    int conflicts;
    int depth;
    bool in_openlist;
    int goal_id; 

    struct compare_node
    {
        bool operator()(const AstarNode* n1, const AstarNode* n2) const
        {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return rand() % 2 == 0;  
            return n1->g_val + n1->h_val > n2->g_val + n2->h_val; //去掉了 = ,有问题再改
        }
    }; 

    struct secondary_compare_node
    {
        bool operator()(const AstarNode* n1, const AstarNode* n2) const
        {
            if (n1->conflicts == n2->conflicts)
            {
                return n1->g_val <= n2->g_val;  
            }
            return n1->conflicts > n2->conflicts;  
        }
    };  

    fibonacci_heap< AstarNode*, compare<AstarNode::compare_node> >::handle_type open_handle;
    fibonacci_heap< AstarNode*, compare<AstarNode::secondary_compare_node> >::handle_type focal_handle;

        struct EqNode
    {
        bool operator() (const AstarNode* n1, const AstarNode* n2) const
        {
            return (n1 == n2) ||
                   (n1 && n2 && n1->xy == n2->xy && n1->goal_id == n2->goal_id); //这里直接比较xy不知道会不会出问题
        }
    };

    struct Hasher
    {
        std::size_t operator()(const AstarNode* n) const
        {
            size_t x_hash = std::hash<int>()(n->xy.first);
            size_t y_hash = std::hash<int>()(n->xy.second);
            return (x_hash ^ y_hash << 1);
        }
    };

    AstarNode(): g_val(0), h_val(0), parent(nullptr), conflicts(0), depth(0), in_openlist(false), goal_id(0) {}
    AstarNode(const coordinate& xy, double g_val, double h_val, AstarNode* parent, int conflicts):
        xy(xy), g_val(g_val), h_val(h_val), parent(parent), conflicts(conflicts), in_openlist(false)
    {
        if(parent != nullptr)
        {
            depth = parent->depth + 1;
            goal_id = parent->goal_id;
        }
        else
        {
            depth = 0;
            goal_id = 0;
        }
    }
    

    inline double getFVal() const { return g_val + h_val; }
};
