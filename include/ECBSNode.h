#ifndef ECBSNODE_H
#define ECBSNODE_H

#include "Common.h"

class ECBSNode{

public:

    struct compare_node
    {
        bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
        {
            return n1->f_val_ >= n2->f_val_;
            // return n1->min_f_val_ >= n2->min_f_val_;
        }
    };

    struct secondary_compare_node
    {
        bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
        {
            if(n1->num_of_collisions_ == n2->num_of_collisions_)
            {
                return n1->f_val_ >= n2->f_val_;
            }
            return n1->num_of_collisions_ > n2->num_of_collisions_;
        }
    };

    typedef fibonacci_heap< ECBSNode*, compare<ECBSNode::compare_node> >::handle_type open_handle_t;
    typedef fibonacci_heap< ECBSNode*, compare<ECBSNode::secondary_compare_node> >::handle_type focal_handle_t;
    open_handle_t open_handle;
    focal_handle_t focal_handle;

    ECBSNode(): parent_(nullptr), g_val_(0), h_val_(0), min_f_val_(0){}
    ECBSNode(ECBSNode* parent);
    ~ECBSNode() = default;
    bool IsSideConfEmpty(){return sideConflicts_.empty();}

    int constraintID_;
    vector<sideConflict> sideConflicts_;
    vector<vertexConflict> vertexConflicts_;

    vector<Path> paths_;//double都为代价
    vector<double> path_cost_;
    vector<sidePath> spaths_;//tuple[0]：正反车标志位,
    vector<Path> turnPoints; //各路径转弯拐点//TODO：在每次计算路径的时候都要记录以下    

    ECBSNode* parent_;
    double g_val_;
    double h_val_;
    double f_val_;
    double min_f_val_;
    size_t depth_; // depth of this CT node
    int num_of_collisions_; // number of conflicts in the current paths
    bool in_openlist_;

private:

};

#endif