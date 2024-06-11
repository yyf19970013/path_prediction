#include "Common.h"

Path Astar::run(const map& map, const coordinate& start, const coordinate& goal)
{
    vector<coordinate> CornerP; // 记录拐点
    
    double h_val = compute_h_value(start, goal);
    if(h_val >= INT_MAX)
    {
        cout << "invalid start or goal!" << endl;
        return Path();
    }

    AstarNode* root = new AstarNode(start, 0, h_val, nullptr, 0);
    root->open_handle = open_list.push(root);
    root->in_openlist = true;
    allNodes_table.insert(root);
    min_f_val = root->getFVal();
    double lower_bound = min_f_val;

    while(!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        curr->in_openlist = false;

        if(curr->xy.first == goal.first && curr->xy.second == goal.second)
        {
            Path path = updatePath(curr);
            releaseAllNodes();
            open_list.clear();
            return path;
        }

        for(const auto& neighbor : getNeighbors(map, curr))
        {
            int next_g_val = curr->g_val + get_w_val(map, neighbor);
            int next_h_val = compute_h_value(neighbor, goal);
            if(next_g_val >= INT_MAX) continue;

            auto next = new AstarNode(neighbor, next_g_val, next_h_val, curr, curr->depth + 1);

            auto it = allNodes_table.find(next);
            if(it == allNodes_table.end()) // 此节点未被拓展过
            {
                next->open_handle = open_list.push(next);
                next->in_openlist = true;
                allNodes_table.insert(next);
            }
            else
            {
                AstarNode* exist_next = *it; // 拿出已拓展过的节点信息
                if(exist_next->in_openlist)
                {
                    if(exist_next->getFVal() > next_g_val + next_h_val)
                    {
                        exist_next->g_val = next_g_val;
                        exist_next->h_val = next_h_val;
                        exist_next->parent = curr;
                        exist_next->depth = next->depth;
                        open_list.increase(exist_next->open_handle);
                    }
                }
                else
                {
                    if(exist_next->getFVal() > next_g_val + next_h_val)
                    {
                        exist_next->in_openlist = true;
                        exist_next->g_val = next_g_val;
                        exist_next->h_val = next_h_val;
                        exist_next->parent = curr;
                        exist_next->depth = next->depth;
                        open_list.push(exist_next);
                    }
                }
                delete(next);
            }
        }
    }

    // No path found
    releaseAllNodes();
    open_list.clear();
    return Path();
}

void Astar::releaseAllNodes()
{
    for (auto it : allNodes_table)
        delete it;
    allNodes_table.clear();
}
