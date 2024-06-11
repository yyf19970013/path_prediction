#include "Astar.h"

// vector<vector<pair<int,int>>> map
// pair<int,int> coordinate;

Path Astar::run(const map& map, const coordinate& start, const coordinate& goal)
{
    vector<coordinate> CornerP; //记录拐点（暂时还没用到）
    
    double h_val = compute_h_value(start, goal);
    if(h_val >= INT_MAX)
    {
        cout << "invaild start or goal!" << endl;
        return Path();
    }
    
    AstarNode* root = new AstarNode(start, 0, h_val, nullptr, 0);

    //存在一个节点自增变量
    root->open_handle = open_list.push(root);
    root->in_openlist = true;
    allNodes_table.insert(root);

    while(!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        open_list.pop();
        curr->in_openlist = false;
        //节点自增变量
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

            auto next = new AstarNode(neighbor, next_g_val, next_h_val, curr, 0);

            auto it = allNodes_table.find(next);
            if(it == allNodes_table.end())//此节点未被拓展过
            {
                next->open_handle = open_list.push(next);
                next->in_openlist = true;
                allNodes_table.insert(next);
            }
            else
            {
                AstarNode* exist_next = *it;//拿出已拓展过的节点信息
                if(exist_next->in_openlist)
                {
                    if(exist_next->getFVal() > next_g_val + next_h_val)
                    {
                        exist_next->g_val = next_g_val;
                        exist_next->h_val = next_h_val;
                        exist_next->parent = curr;
                        open_list.increase(exist_next->open_handle);
                    }
                }
                // else
                // {
                //     if(exist_next->getFVal() > next_g_val + next_h_val)
                //     {
                //         exist_next->in_openlist = true;
                //         exist_next->g_val = next_g_val;
                //         exist_next->h_val = next_h_val;
                //         exist_next->parent = curr;
                //         open_list.increase(exist_next->open_handle);
                //     }
                // }
                delete(next);
            }
        }
        releaseAllNodes();
        open_list.clear();

        return Path();
    }
}













Path Astar::run(const map& map, const coordinate& start, const coordinate& goal, ReservationTable& rt){}


Path Astar::updatePath(const AstarNode* goal)
{
    Path path;
    path_cost = goal->getFVal();
    //是否记录单条路径碰撞
    const AstarNode* curr = goal;
    while(curr != nullptr)
    {
        path.push_back(curr->xy);
        curr = curr->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void Astar::releaseAllNodes()
{
    for(auto it : allNodes_table)
    {
        delete it;
    }
    allNodes_table.clear();
}

list<coordinate> Astar::getNeighbors(const map& map, const AstarNode* node)
{   
    list<coordinate> neighbors;
    coordinate curr_xy = node->xy;
    mapRoute maproute = (map.at(curr_xy.first).at(curr_xy.second)).second;
    for(int i = 0; i < 4; ++i)
    {
        if(i == 0 && curr_xy.first - 1 >= 0 && get<0>(maproute) == 0)
        {
            neighbors.emplace_back(make_pair(curr_xy.first - 1, curr_xy.second));
         }
        else if(i == 1 && curr_xy.second + 1 <= common_.getCol() - 1  && get<1>(maproute) == 0)
        {
            neighbors.emplace_back(make_pair(curr_xy.first, curr_xy.second + 1));
        }
        else if(i == 2 && curr_xy.first + 1 <= common_.getRow() - 1 && get<2>(maproute) == 0)
        {
            neighbors.emplace_back(make_pair(curr_xy.first + 1, curr_xy.second));
        }
        else if(i == 3 && curr_xy.second - 1 >= 0 && get<3>(maproute) == 0)
        {
            neighbors.emplace_back(make_pair(curr_xy.first, curr_xy.second - 1));
        }
    }
    return neighbors;
}

int Astar::get_w_val(const map& map, const coordinate& next)
{
    if(map.at(next.first).at(next.second).first == 1) return INT_MAX;
    return 1;
}

sidePath Astar::updateSidePath(const Path& path)
{
    sidePath spath;
    size_t len = path.size();
    vector<size_t> conner_index;
    assert(len > 0);
    for(int i = 1; i < len - 1; ++i)
    {
        if(path.at(i-1).first == path.at(i).first == path.at(i+1).first ||
           path.at(i-1).second == path.at(i).second == path.at(i+1).second) continue;
        conner_index.push_back(i);
    }

    size_t conner_num = conner_index.size();

    if(conner_num == 0)//无转折点
    {
        spath.emplace_back(make_pair(path.begin(), path.end()));
    }

    spath.emplace_back(make_pair(path.at(0), path.at(conner_index.at(0)-1)));//第一段
    for(int i = 0; i < conner_index.size() - 1; ++i)
    {
        spath.emplace_back(make_pair(path.at(conner_index.at(i)), path.at(conner_index.at(i+1) - 1)));
    }
    spath.emplace_back(make_pair(path.at(conner_index.at(conner_num - 1)), path.at(len - 1)));//最后一段
    return spath;
}