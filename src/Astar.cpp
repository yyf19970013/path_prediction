#include "Astar.h"

// vector<vector<pair<int,int>>> map
// pair<int,int> coordinate;

Path Astar::run(const Map& map, const coordinate& start, const coordinate& goal)
{
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
    int step = 0;
    while(!open_list.empty())
    {
        AstarNode* curr = open_list.top();
        // if(step >0)
        // {
        //     cout << "step:" << step << "" << curr->parent->coord.x << " " << curr->parent->coord.y << endl;
        // }
        
        open_list.pop();
        curr->in_openlist = false;
        //节点自增变量
        if(curr->coord == goal)
        {
            Path path = updatePath(curr);
            releaseAllNodes();
            open_list.clear();
            return path;
        }
        list<coordinate> cs = getNeighbors(map, curr);
        // cout << "cs.size:" << cs.size() << endl;
        for(const auto& neighbor : cs)//这里计算的就是能走的邻居
        {
            
            /*
            存在到处走不通的
            */
            int next_g_val = curr->g_val + 1;
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
                if(exist_next->getFVal() > next_g_val + next_h_val)
                {
                    exist_next->g_val = next_g_val;
                    exist_next->h_val = next_h_val;
                    exist_next->parent = curr;
                    open_list.increase(exist_next->open_handle);
                }
                delete(next);//到这步说明已存在的节点val < 当前next的拓展值
            }
        }
        step++;
    }
    releaseAllNodes();
    open_list.clear();
    return Path();
}

Path Astar::run(const Map& map, const coordinate& start, const coordinate& goal, ConstraintTable& ct, int agvClass)
{
    double h_val = compute_h_value(start, goal);
    if(h_val >= INT_MAX)
    {
        cout << "invaild start or goal!" << endl;
        return Path();
    }
    
    AstarNode* root = new AstarNode(start, 0, h_val, nullptr, 0);
    
    vector<vector<pointConstraint>> ct_;
    if(agvClass) ct_ = ct.getPostiveCT();
    else ct_ = ct.getReverseCT();

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
        if(curr->coord.x == goal.x && curr->coord.y == goal.y)
        {
            Path path = updatePath(curr);
            releaseAllNodes();
            open_list.clear();
            return path;
        }
        for(const auto& neighbor : getNeighbors(map, curr))
        {
            pair<double,double> neighbor_time = oi.GetTime(start, neighbor);
            for(const auto& t : ct_.at(neighbor.x).at(neighbor.y))
            {
                if(common_.isTimeOverlap(t, neighbor_time))
                    continue;
            }
            int next_g_val = curr->g_val + 1;
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
                delete(next);
            }
        }
    }
    releaseAllNodes();
    open_list.clear();
    return Path();
}

Path Astar::updatePath(const AstarNode* goal)
{
    Path path;
    //是否记录单条路径碰撞
    const AstarNode* curr = goal;
    while(curr != nullptr)
    {
        path.push_back(curr->coord);
        curr = curr->parent;
    }
    path_cost = path.size();
    std::reverse(path.begin(), path.end());
    return path;
}

void Astar::releaseAllNodes()
{
    for(auto it : allNodes_table)
        delete it;
    allNodes_table.clear();
}

list<coordinate> Astar::getNeighbors(const Map& map, const AstarNode* node)
{   
    list<coordinate> neighbors;
    coordinate curr_coordinate = node->coord;
    mapRoute maproute = (map.at(curr_coordinate.x).at(curr_coordinate.y));
    for(int i = 0; i < 4; ++i)
    {
        if(i == 0 && curr_coordinate.x - 1 >= 0 && get<0>(maproute) == true)
        {
            neighbors.emplace_back(coordinate(curr_coordinate.x - 1, curr_coordinate.y));
         }
        else if(i == 1 && curr_coordinate.y + 1 <= col_ - 1  && get<1>(maproute) == true)
        {
            neighbors.emplace_back(coordinate(curr_coordinate.x, curr_coordinate.y + 1));
        }
        else if(i == 2 && curr_coordinate.x + 1 <= row_ - 1 && get<2>(maproute) == true)
        {
            neighbors.emplace_back(coordinate(curr_coordinate.x + 1, curr_coordinate.y));
        }
        else if(i == 3 && curr_coordinate.y - 1 >= 0 && get<3>(maproute) == true)
        {
            neighbors.emplace_back(coordinate(curr_coordinate.x, curr_coordinate.y - 1));
        }
    }
    return neighbors;
}

// int Astar::get_w_val(const Map& map, const coordinate& next)//TODO：运行的时候已经提前知道是否能够通行，不需要再辨别
// {
//     // if(map.at(next.x).at(next.y).first == 1) return INT_MAX;
//     // return 1;
// }

sidePath Astar::updateSidePath(const Path& path, const vector<int>& conner_index)
{
    sidePath spath;
    size_t len = path.size();
    // assert(len > 1);
    if(len == 2)
    {
        spath.emplace_back(path);
        return spath;
    }
    int conner_num = conner_index.size();
//---------------------------------------
    if(conner_num == 0){//0转折点
        spath.push_back(path);
        return spath;
    }

    if(conner_num == 1)//1转折点
    {
        for(size_t i = 0; i < 2; ++i)
        {
            Path localPath;
            size_t idx = conner_index.front();
            if(i == 0)
            {
                for(size_t j = 0; j < idx; ++j)
                    localPath.push_back(path.at(j));
                spath.push_back(localPath);
            }
            else
            {
                for(size_t j = idx; j < len; ++j)
                    localPath.push_back(path.at(j));
                spath.push_back(localPath);
            }
        }
        return spath;
    }
    for(size_t i = 0; i < conner_num - 1; ++i)
    {
        Path localPath;
        size_t idx1 = conner_index.at(i);
        size_t idx2 = conner_index.at(i+1);
        if(i == 0)
        {
            for(int j = 0; j < idx1; ++j)
                localPath.push_back(path.at(j));
            spath.push_back(localPath);
            localPath.clear();         
            for(int j = idx1;j < idx2; ++j)
                localPath.push_back(path.at(j));
            spath.push_back(localPath);
            continue;
        }
        for(int j = idx1; j < idx2; ++j)
            localPath.push_back(path.at(j));
        spath.push_back(localPath);
    }
    Path localPath;
    for(size_t i = conner_index.back(); i < len; ++i)
        localPath.push_back(path.at(i));
    spath.push_back(localPath);
    return spath;
}

void Astar::setMapsize(Map map)
{
    row_ = map.size();
    col_ = map.at(0).size();
}

void Astar::drawPath(const Path& p, const vector<vector<int>>& m)
{
    vector<vector<char>> view(20,vector<char>(20,'.'));
    for(const auto& c : p)
    {
        view.at(c.x).at(c.y) = '#';
    }
    for(int i = 0; i < row_; ++i)
    {
        for(int j = 0; j < col_; ++j)
        {
            if(m.at(i).at(j) == 1)
                view.at(i).at(j) = '@';
        }
    }
    for(int i = 0; i < row_; ++i)
    {
        for(int j = 0; j < col_; ++j)
        {
            cout << view.at(i).at(j) << " ";
        }
        cout << endl;
    }
}










