#include "ECBS.h"

ECBS::ECBS(){;}

bool ECBS::run(const vector<tuple<int,int,int,int>>& starts,
             const vector<tuple<int,int,int,int>>& goals,
             int time_limits)

{
    this->starts_ = starts;
    this->goals_ = goals;
    this->num_of_agents_ = starts.size();//可能要更改（starts里可能只放有任务的或者放所有的）

    if(!generate_root_node()) return false;

    while(!open_list_.empty() && !solution_found_)
    {
        ECBSNode* curr = pop_node(); // 待开发

        if(curr->IsConflictsEmpty())
        {
            solution_found_ = true;
            break;
        }

        update_paths(curr);// 待开发

        get_w(curr);// 待开发

        choose_conflict(*curr);// 待开发

        ECBSNode* n[2];
        for(int i = 0; i < 2; ++i)
        {
            n[i] = new ECBSNode(curr);
            resolve_conflict(curr->conflict_, n[0], n[1]);//只有需要传入他的指针引用，才加入 * 符号
        }

        vector<pair<int, Path>> copy(curr->paths_);
        for(int i = 0; i < 2; ++i) 
        {
            bool sol = generate_child(n[i], curr);
            if(sol)
            {
                push_node(n[i]);
            }
            else
            {
                delete(n[i]);
                n[i] = nullptr;
            }
            curr->paths_ = copy;
        }
    }
    get_solution();
    cout << "OK!" << endl;
}

bool ECBS::generate_root_node()
{
    dummy_start_ = new ECBSNode();
    dummy_start_->paths_.resize(num_of_agents_, make_pair(0, Path()));

    for(int i = 0; i < num_of_agents_; ++i)
    {
        /*
        这里添加一个根据车辆状态选择地图的逻辑待添加
        */
        map map = maps_[0];

        coordinate start, goal;
        start.first = get<1>(starts_.at(i));
        start.second = get<2>(starts_.at(i));
        goal.first = get<1>(goals_.at(i));
        goal.second = get<2>(goals_.at(i));

        /*
        初次进入不需要rt表格信息，后面才需要
        */

        Path path = Astar_.run(map, start, goal);
        sidePath spath = Astar_.updateSidePath(path); //段路径

        if(path.empty())
        {
            cout << "No solution!" << endl;
            return false;
        }
        /*
        记录节点扩展信息部分待添加
        */
        dummy_start_->spaths_.push_back(make_tuple(i, spath, Astar_.min_f_val, Astar_.path_cost));//min_f_val 和 path_cost 还没更新
        dummy_start_->paths_.push_back(make_pair(i, path));
        /*
        记录节点扩展信息部分待添加  
        */
    }
    /*
    这里也可能要加一段，来指定我们查找冲突的范围，根据最短段的长度
    */
    int min_len = INT_MAX;//用来记录遍历范围

    for(const auto& p : dummy_start_->spaths_)
    {
        int x = abs(get<1>(p).at(0).first.first - get<1>(p).at(0).second.first);
        int y = abs(get<1>(p).at(0).first.second - get<1>(p).at(0).second.second);
        int len = abs(x - y) + 1;
        min_len = min(min_len, len);  
    }

    // list<shared_ptr<Conflict>> conf = dummy_start_->conflicts_;//记得在后面需要将conf重新写入dummy_starts;
    find_confilcts(min_len, dummy_start_);
    // dummy_start_->setConfilcts(conf);//这里已写入

    push_node(dummy_start_);
    best_node_ = dummy_start_;

    
}

void ECBS::find_confilcts(int len, ECBSNode* node)
{   
    int path_num = node->spaths_.size();
    for(int i = 0; i < path_num - 2; ++i)
    {
        if(get<2>(node->spaths_.at(i)) == 0) continue;//如果不存在路径跳过
        sidePath spi = get<1>(node->spaths_.at(i));// A车

        coordinate spi_head = spi.at(0).first;
        coordinate spi_end = spi.at(0).second;

        bool x_driect = spi_head.first == spi_end.first ? true : false;//记录其方向
        
        coordinate veci = make_pair(spi_end.first - spi_head.first, spi_end.second - spi_end.second);
        for(int j = 1; j < path_num - 1; ++j)
        {
            if(get<2>(node->spaths_.at(j)) == 0) continue;
            sidePath spj = get<1>(node->spaths_.at(j)); //B车

            coordinate spj_head = spj.at(0).first;
            coordinate spj_end = spj.at(0).second;

            coordinate vecj = make_pair(spj_end.first - spj_head.first, spj_end.second - spj_end.second);
            int dot_res = (veci.first * vecj.first) + (veci.second * vecj.second);
            if(dot_res == 0)//点碰撞可能性
            {
                pair<coordinate,coordinate> line_x = x_driect == true ? spi.at(0) : spj.at(0);//可以融合
                pair<coordinate,coordinate> line_y = line_x == spi.at(0) ? spj.at(0) : spi.at(0);//可以融合

                coordinate interP = getIntersectionPoint(line_x, line_y);//计算交点
                pair<double,double> x_time_range = oi.GetTime(line_x.first, interP);
                pair<double,double> y_time_range = oi.GetTime(line_y.first, interP);
                
            }
            else//边碰撞可能性
            {
                if(x_driect)//横向
                {
                    if(abs(spi_head.first - spj_head.first) > 2) continue;//无碰撞可能性
                    
                }
            }
        }
    }
}

void ECBS::find_conflict(int len, const Path& a, const Path& b, bool a_v, bool b_v)
{

}

coordinate ECBS::getIntersectionPoint(const pair<coordinate,coordinate>& line_x, const pair<coordinate,coordinate>& line_y)
{
    coordinate coord = make_pair(line_x.first.first, line_y.first.second);
    if(abs(coord.second - line_x.first.second) <= abs(line_x.second.second - line_x.first.second) &&
       abs(coord.first - line_y.first.first) <= abs(line_y.second.first - line_y.second.second))
        return coord;
    return coordinate();
}
