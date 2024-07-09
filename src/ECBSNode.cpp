#include "ECBSNode.h"

ECBSNode::ECBSNode(ECBSNode* parent){
    parent_ = parent;
    g_val_ = parent->g_val_;//这个赋值后面再看一下
    h_val_ = 0;
    min_f_val_ = parent->min_f_val_;
    depth_ = parent_->depth_ + 1;
}

// void ECBSNode::setPaths(int i, const sidePath& path, double min_f_val, double cost)
// {
//     paths_.emplace_back(make_tuple(i, path, min_f_val, cost));
// }