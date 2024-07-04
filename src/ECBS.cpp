#include "ECBS.h"

void ECBS::initialize(vector<Map>& maps, vector<pair<coordinate, int>>& starts, vector<coordinate>& ends, int min_len, double turntime)
{
    maps_ = maps;
    starts_ = starts;
    ends_ = ends;
    min_len_ = min_len;
    turntime_ = turntime;
    num_of_agents_ = starts_.size();
}

bool ECBS::run()
{
    if(!generate_root_node()) return false;

    while(!open_list_.empty() && !solution_found_)
    {
        random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(1,1000);
        this->randomNum_ = dis(gen);

        ECBSNode* curr = open_list_.top();
        open_list_.pop();
        curr->in_openlist_ = false;

        update_paths(curr); 

        if(curr->IsSideConfEmpty())//加点冲突计划
        {
            vector<bool> updated(num_of_agents_, false);
            resolveVertexConf(curr, updated);
            for(int i = 0; i < updated.size(); ++i)
            {
                if(updated.at(i) == true) continue;
                for(int j = 0; j < min_len_; ++j)
                    runPaths_.at(i).emplace_back(paths_.at(i).at(j));
            }
            solution_found_ = true;
        }

        if(!solution_found_)
        {
            //TODO:这里加一个best_node 赋值模块

            ECBSNode* n[2];
            for(int i = 0; i < 2; ++i)
            {
                n[i] = new ECBSNode(curr);
                resolve_sideConflict(n[0], n[1]);
            }

            vector<Path> copy(curr->paths_);

            for(int i = 0; i < 2; ++i) 
            {
                bool sol = generate_child(n[i]);
                if(sol)
                    n[i]->open_handle = open_list_.push(n[i]);
                else
                {
                    delete(n[i]);
                    n[i] = nullptr;
                }
                curr->paths_ = copy;
            }
        }
    }
    return solution_found_;
}

bool ECBS::generate_root_node()
{
    dummy_start_ = new ECBSNode();
    for(size_t i = 0; i < num_of_agents_; ++i)
    {
        if(ends_.at(i) == coordinate()) continue;
        Map map = maps_.at(starts_.at(i).second);
        coordinate start, end;
        start.x = starts_.at(i).first.x;
        start.y = starts_.at(i).first.y;
        end.x = ends_.at(i).x;
        end.y = ends_.at(i).y;

        Path path = Astar_.run(map, start, end);
        vector<size_t> conner_idx;//用以记录转弯节点下标(不包括start和end)
        // path = minTurnsPath(path, conner_idx);//TODO：根据现有路径变化进行方法更新
        getConneridx(path, conner_idx);//临时
        sidePath spath = Astar_.updateSidePath(path, conner_idx); //段路径生成
        dummy_start_->f_val_ += Astar_.path_cost;//TODO：添加非伯纳切判断值
        dummy_start_->path_cost_.push_back(Astar_.path_cost);
        vector<coordinate> turnspoint;//记录转弯点（不记录start和end）
        if(conner_idx.size() != 0)
        {
            for(size_t j = 0; j < conner_idx.size(); ++j)
                turnspoint.emplace_back(path.at(conner_idx.at(j)));
            dummy_start_->turnPoints.emplace_back(turnspoint);
        }

        if(path.empty())
        {
            cout << "No solution!" << endl;
            return false;
        }
        dummy_start_->spaths_.push_back(spath);//path_cost记录
        dummy_start_->paths_.push_back(path);
    }
    
    for(const auto& p : dummy_start_->spaths_)//TODO:涉及到下次更新如何去一直更新，可能需要将路径在ECBS模块中进行随动
    {
        int len = p.at(0).size();
        min_len_ = min(min_len_, len);  
    }

    // list<shared_ptr<Conflict>> conf = dummy_start_->conflicts_;//记得在后面需要将conf重新写入dummy_starts;
    find_sideConflicts(min_len_, dummy_start_);
    // dummy_start_->setConfilcts(conf);//这里已写入
    dummy_start_->open_handle = open_list_.push(dummy_start_);
    best_node_ = dummy_start_;
    return true;
}

void ECBS::find_sideConflicts(int len, ECBSNode* node)
{   
    if(len == 1)
    {
        for(int i = 0; i < num_of_agents_ - 1; ++i)
        {
            if(node->paths_.at(i).size() == 0) continue;
            int i_ori = starts_.at(i).second % 2 == 0 ? 0 : 1;
            coordinate i_coord = node->paths_.at(i).front();
            for(int j = 1; j < num_of_agents_ ; ++j)
            {
                Path cs;
                if(node->paths_.at(i).size() == 0) continue;
                int j_ori = starts_.at(j).second % 2 == 0 ? 0 : 1;
                coordinate j_coord = node->paths_.at(j).front();
                if(i_ori == j_ori)
                {
                    if(i_coord.x-1 >= 0) cs.emplace_back(coordinate(i_coord.x-1, i_coord.y));
                    if(i_coord.x+1 < row_) cs.emplace_back(coordinate(i_coord.x+1, i_coord.y));
                    for(coordinate c : cs)
                        if(c == j_coord) vertexConflicts.emplace_back(i, j, i_coord, j_coord);
                }
                else if (i_ori == 1)//j_ori == 0
                {
                    if(i_coord.x-1 >= 0) cs.emplace_back(coordinate(i_coord.x-1, i_coord.y));
                    if(i_coord.x+1 < row_) cs.emplace_back(coordinate(i_coord.x+1, i_coord.y));
                    if(i_coord.x+2 < row_) cs.emplace_back(coordinate(i_coord.x+2, i_coord.y));
                    for(coordinate c : cs)
                        if(c == j_coord) vertexConflicts.emplace_back(i, j, i_coord, j_coord);
                }
                else
                {
                    if(i_coord.x-1 >= 0) cs.emplace_back(coordinate(i_coord.x-1, i_coord.y));
                    if(i_coord.x-2 >= 0) cs.emplace_back(coordinate(i_coord.x+1, i_coord.y));
                    if(i_coord.x+1 < row_) cs.emplace_back(coordinate(i_coord.x+2, i_coord.y));
                    for(coordinate c : cs)
                        if(c == j_coord) vertexConflicts.emplace_back(i, j, i_coord, j_coord);
                }   
            } 
        }
        return;
    }
    for(int i = 0; i < num_of_agents_ - 1; ++i)
    {  
        if(node->paths_.at(i).size() == 0) continue;//如果不存在路径跳过

        int i_ori = starts_.at(i).second % 2 == 0 ? 0 : 1; //获取正反车信息(1:正车，0：反车)
        vector<coordinate> iLine(len);

        for(int k = 0; k < len; ++k)
            iLine.push_back(node->paths_.at(i).at(k));
        
        bool i_x_driect = iLine.front().x == iLine.back().x ? true : false;//记录横纵向 t->横 f->竖
        coordinate veci = {iLine.back().x - iLine.front().x, iLine.back().y - iLine.back().y};//A车向量

        for(int j = 1; j < num_of_agents_; ++j)
        {
            if((node->paths_.at(j).size()) == 0) continue;

            int j_ori = starts_.at(j).second % 2 == 0 ? 0 : 1; //获取B车正反车信息
            vector<coordinate> jLine(len);

            for(int k = 0; k < len; ++k)
                jLine.push_back(node->paths_.at(j).at(k));

            bool j_x_driect = iLine.front().x == iLine.back().x ? true : false;//B车横竖
            coordinate vecj = {jLine.at(len-1).x - jLine.at(0).x, jLine.at(len-1).y - jLine.at(len-1).y};//B车向量

            int dot_res = (veci.x * vecj.x) + (veci.y * vecj.y);//计算二者行进情况

            find_sideConflict(iLine, jLine, i_ori, j_ori, i, j, i_x_driect, dot_res, node);//TODO:具体边冲突对应计算
        }
    }
}

coordinate ECBS::getIntersectionPoint(const Path& horizonLine, const Path& vertiLine)
{
    coordinate coord{horizonLine.front().x, vertiLine.front().y};
    int min_x, max_x, min_y, max_y;
    min_x = vertiLine.front().x > vertiLine.back().x ? vertiLine.back().x : vertiLine.front().x;
    max_x = min_x == vertiLine.front().x ? vertiLine.back().x : vertiLine.front().x;
    min_y = horizonLine.front().y > horizonLine.back().y ? horizonLine.back().y : horizonLine.front().y;
    max_y = min_y == horizonLine.front().y ? horizonLine.back().y : horizonLine.front().y;
    if((coord.x >= min_x && coord.x <= max_x) ||
        coord.y >= min_y && coord.y >= max_y)
        return coord;
    return coordinate();
}

vector<int> ECBS::getIntersectionLine(const Path& line1, const Path& line2, bool isX)//TODO：需要修改
{   
    if(isX)//横向
    {
        vector<int> ys = {line1.front().y, line1.back().y, line2.front().y, line2.back().y};
        sort(ys.begin(),ys.end());
        if(ys.at(3) - ys.at(0) + 1 >= 
           abs(line1.front().y - line1.back().y) + 
           abs(line2.front().y - line2.back().y) + 2) 
           return vector<int>();
        return ys;
    }
    else
    {
        vector<int> xs{line1.front().x, line1.back().x, line2.front().x, line2.back().x};
        sort(xs.begin(), xs.end());
         if(xs.at(3) - xs.at(0) + 1 >= 
           abs(line1.front().x - line1.back().x) + 
           abs(line2.front().x - line2.back().x) + 2) 
           return vector<int>();
        return xs;
    }
}

// Path ECBS::minTurnsPath(const Path& path, vector<size_t> conner_idx, int agvClass)
// {   
//     Path resPath;
//     size_t sp = 0;
//     vector<coordinate> conner_coord;
//     int len = path.size();
//     if(len <= 3) return path;
//     conner_coord.push_back(path.front());
//     for(size_t i = 1; i < len - 1; ++i)
//     {
//         if(path.at(i-1).x == path.at(i).x == path.at(i+1).x ||
//            path.at(i-1).y == path.at(i).y == path.at(i+1).y) continue;
//         conner_coord.push_back(path.at(i));
//         conner_idx.push_back(i);
//     }
//     conner_coord.push_back(path.at(len-1));

//     for(size_t sp = 0; sp < conner_coord.size() - 3; ++sp)
//     {
//         coordinate c1 = conner_coord.at(sp);
//         coordinate c2 = conner_coord.at(sp + 1);
//         coordinate c3 = conner_coord.at(sp + 2);
//         coordinate c2_;
//         bool xdriction = c2.x - c1.x == 0 ? true : false;//沿x or y
//         if(xdriction)//横
//         {
//             c2_.x = c3.x;
//             c2_.y = c1.y;
//             Path path1 = connectPath(c1, c2_);
//             Path path2 = connectPath(c2_, c3);
//             if(isCrossObs(path1,agvClass) || isCrossObs(path2,agvClass)) continue;
//             else
//             {
//                 c2 = c2_;
//                 if(c3.x == conner_coord.back().x && c3.y == conner_coord.back().y) break;
//                 c3 = conner_coord.at(sp + 3);
//                 for(size_t i = sp + 3; i < conner_coord.size() - 1; ++i)
//                     conner_coord.at(i) = conner_coord.at(i+1);
//                 conner_coord.pop_back();
//                 sp--;
//             }
//         }
//         else//竖
//         {
//             c2_.x = c1.x;
//             c2_.y = c3.y;
//             Path path1 = connectPath(c1, c2_);
//             Path path2 = connectPath(c2_, c3);
//             if(isCrossObs(path1,agvClass) || isCrossObs(path2,agvClass)) continue;
//             else
//             {
//                 c2 = c2_;
//                 if(c3.x == conner_coord.back().x && c3.y == conner_coord.back().y) break;
//                 c3 = conner_coord.at(sp + 3);
//                 for(size_t i = sp + 3; i < conner_coord.size() - 1; ++i)
//                     conner_coord.at(i) = conner_coord.at(i+1);
//                 conner_coord.pop_back();
//                 sp--;
//             }
//         }
//     }
//     return getPointPath(conner_coord);
// }

// bool ECBS::isCrossObs(const Path& path, int ID)//ID:1、2、3、4
// {
//     Map m = maps_.at(ID);
//     for(const auto& c : path)
//     {
//         if(m.at(c.x).at(c.y) == 1)
//     }
// }

// Path ECBS::connectPath(const coordinate& start, const coordinate& end)
// {
//     Path p;//依旧左闭右开
//     if(start.x == end.x && start.y > end.y)// 平行且向左延伸
//     {
//         int y = start.y;
//         while(y > end.y)
//         {
//             p.emplace_back(coordinate(start.x, y));
//             y--;
//         }
//     }
//     else if(start.x == end.x && start.y < end.y)//平行向右延伸
//     {
//         int y = start.y;
//         while(y < end.y)
//         {
//             p.emplace_back(coordinate(start.x, y));
//             y++;
//         }
//     }
//     else if(start.y == end.y && start.x > end.x)//垂直向上延伸
//     {
//         int x = start.x;
//         while(x > end.x)
//         {
//             p.emplace_back(coordinate(x, start.y));
//             x--;
//         }
//     }
//     else if(start.y == end.y && start.x < end.x)//垂直向下延伸
//     {
//         int x = start.x;
//         while(x < end.x)
//         {
//             p.emplace_back(coordinate(x, start.y));
//             x++;
//         }
//     }
//     return p;
// }

// Path ECBS::getPointPath(const vector<coordinate>& turns)
// {
//     Path resPath;
//     for(size_t i = 0; i < turns.size() - 2; ++i)
//     {
//         coordinate c1 = turns.at(i);
//         coordinate c2 = turns.at(i+1);
//         if(c1.x == c2.x)//横
//         {
//             if(c1.y > c2.y)
//             {
//                 for(int j = c1.y; j > c2.y; --j) 
//                     resPath.emplace_back(coordinate{c1.x,j});
//             }
//             else
//             {
//                 for(int j = c1.y; j < c2.y; ++j) 
//                     resPath.emplace_back(coordinate{c1.x,j});
//             }
//         }
//         else//竖
//         {
//             if(c1.x > c2.x)
//             {
//                 for(int j = c1.x; j > c2.x; --j)
//                     resPath.emplace_back(coordinate{j,c1.y});
//             }
//             else
//             {
//                 for(int j = c1.x; j < c2.x; ++j)
//                     resPath.emplace_back(coordinate{j,c1.y});
//             }
//         }
//     }
//     resPath.push_back(turns.at(turns.size() - 1));
//     return resPath;
// }
void ECBS::getConneridx(const Path& path, vector<size_t>& cs)
{
    int len = path.size();
    if(len <= 2) return;
    for(size_t i = 1; i < len - 1; ++i)
    {
        coordinate BA(path.at(i).x - path.at(i-1).x, path.at(i).y - path.at(i-1).y);
        coordinate BC(path.at(i+1).x - path.at(i).x, path.at(i+1).y - path.at(i).y);
        int pdot = BA.x * BC.x + BA.y * BC.y;
        if(pdot != 0) continue;
        else
            cs.push_back(i);
    }
}

void ECBS::find_vertexConflicts(int len, ECBSNode* node)
{
    for(int i = 0; i < num_of_agents_ - 1; ++i)
    {  
        if(node->paths_.at(i).size() == 0) continue;//如果不存在路径跳过

        int i_ori = starts_.at(i).second % 2 == 0 ? 0 : 1; //获取正反车信息(1:正车，0：反车)
        vector<coordinate> iLine(len);

        for(int k = 0; k < len; ++k)
            iLine.push_back(node->paths_.at(i).at(k));
       
        bool i_x_driect = iLine.front().x == iLine.back().x ? true : false;//记录横纵向 t->横 f->竖
        coordinate veci = {iLine.back().x - iLine.front().x, iLine.back().y - iLine.back().y};//A车向量

        for(int j = 1; j < num_of_agents_; ++j)
        {
            if((node->paths_.at(j).size()) == 0) continue;

            int j_ori = starts_.at(j).second % 2 == 0 ? 0 : 1; //获取B车正反车信息
            vector<coordinate> jLine(len);

            for(int k = 0; k < len; ++k)
                jLine.push_back(node->paths_.at(j).at(k));

            bool j_x_driect = iLine.front().x == iLine.back().x ? true : false;//B车横竖
            coordinate vecj = {jLine.at(len-1).x - jLine.at(0).x, jLine.at(len-1).y - jLine.at(len-1).y};//B车向量

            int dot_res = (veci.x * vecj.x) + (veci.y * vecj.y);//获取车辆行驶情况

            if(dot_res == 0)
                find_vertexConflict(iLine, jLine, i_ori, j_ori, i, j, node);//点冲突可能性
            
                //TODO：需要添加一个bool值来调控
        }
    }
}

void ECBS::find_sideConflict(const Path& iLine, const Path& jLine, 
                            int i_ori, int j_ori,
                            int iID, int jID,
                            bool x_driect, int dot_res,
                            ECBSNode* node)
{
    if(dot_res == 0)//点碰撞中存在的需重规划部分
    {
        if((abs(iLine.front().x - jLine.front().x) <= 1) &&
           (iLine.front().y == jLine.front().y)) 
            node->sideConflicts_.emplace_back(make_pair(iID, jID));

        else if((abs(iLine.front().x - jLine.front().x) == 2) &&
                (iLine.front().y == jLine.front().y))
        {
            if((i_ori == 1 && j_ori == 0 && iLine.front().x > jLine.front().x) ||
               (i_ori == 0 && j_ori == 1 && iLine.front().x < jLine.front().x))
                node->sideConflicts_.emplace_back(make_pair(iID, jID));
        }
    }
    else//边冲突
    {
        if(x_driect)//横向
        {
            int x_interval = abs(iLine.front().x - jLine.front().x);//计算横向间隔
            if(x_interval > 2) return;//超过间隔最大碰撞范围
            vector<int> y_inter_range = getIntersectionLine(iLine, jLine, x_driect);//拿到重叠区域Y范围
            if(y_inter_range.empty()) return;
            if(x_interval == 2)
            {
                Path upLine, downLine;
                int upOri, downOri, upID, downID;
                if(iLine.front().x < jLine.front().x)
                {
                    upLine = iLine;
                    downLine = jLine;
                    upOri = i_ori;
                    downOri = j_ori;
                    upID = iID;
                    downID = jID;
                }
                else
                {
                    upLine = jLine;
                    downLine = iLine;
                    upOri = j_ori;
                    downOri = i_ori;
                    upID = jID;
                    downID = iID;
                }
                if(upOri == 1 && downID == 0)
                    node->sideConflicts_.emplace_back(make_pair(upID, downID));
            }
            else//面对间隔为0和1
            {
                if(dot_res == -1)//相对行驶
                    node->sideConflicts_.emplace_back(make_pair(iID, jID));
                else//同
                {
                    if(y_inter_range.at(0) == y_inter_range.at(1))
                    {
                        sideConflict conf = make_pair(iID, jID);
                        node->sideConflicts_.push_back(conf);
                    }
                    /*
                    同向部分依旧是点碰撞解决，所以最后重规划完再考虑
                    */
                //-------------------------------------------
                    // Path lLine, rLine;
                    // int lOri, rOri, lID, rID;
                    // if(x_inter_range.at(0) == iLine.front().x || x_inter_range.at(0) == iLine.back().x)
                    // {
                    //     lLine = iLine;
                    //     rLine = jLine;
                    //     lOri = i_ori;
                    //     rOri = j_ori;
                    //     lID = iID;
                    //     rID = jID;
                    // }
                    // else
                    // {
                    //     lLine = jLine;
                    //     rLine = iLine;
                    //     lOri = j_ori;
                    //     rOri = i_ori;
                    //     lID = jID;
                    //     rID = iID;
                    // }
                    // if(lLine.back().x - lLine.front().x > 0)//向右运动
                    //     vertexConflicts.emplace_back(make_tuple(lID, rID, coordinate(lLine.front().x, rLine.front().y - 1), coordinate()));
                    // else//向左运动
                    //     vertexConflicts.emplace_back(make_tuple(rID, lID, coordinate(rLine.front().x, lLine.front().y + 1), coordinate()));
                }
            }
        }        
        else//纵向
        {
            if(iLine.front().y != jLine.front().y) return;
            vector<int> x_overlap_range = getIntersectionLine(iLine, jLine, x_driect);//获取交并边
            if(!x_overlap_range.empty())
                node->sideConflicts_.emplace_back(make_pair(iID, jID));
            // Path upLine, downLine;
            // int upOri, downOri, upID, downID;
            // if(x_overlap_range.at(0) == iLine.front().x || x_overlap_range.at(0) == iLine.back().x)
            // {
            //     upLine = iLine;
            //     downLine = jLine;
            //     upOri = i_ori;
            //     downOri = j_ori;
            //     upID = iID;
            //     downID = jID;
            // }
            // else
            // {
            //     upLine = jLine;
            //     downLine = iLine;
            //     upOri = j_ori;
            //     downOri = i_ori;
            //     upID = jID;
            //     downID = iID;
            // }
            // if(dot_res == 1)//同向
            // {
                
                /*
                同向部分依旧是点碰撞解决，所以最后重规划完再考虑
                */
                //-------------------------------------------

                // if(upLine.back().x - upLine.front().x > 0)//往下运动
                // {
                //     if(upOri == 1 && downOri == 0)
                //         vertexConflicts.emplace_back(make_tuple(upID, downID, coordinate(downLine.front().x - 3, downLine.front().y), coordinate()));
                //     else
                //         vertexConflicts.emplace_back(make_tuple(upID, downID, coordinate(downLine.front().x - 2, downLine.front().y), coordinate()));
                // }
                // else//往上运动
                // {
                //     if(upOri == 1 && downOri == 0)
                //         vertexConflicts.emplace_back(make_tuple(downID, upID, coordinate(upLine.front().x + 3, upLine.front().y), coordinate()));
                //     else
                //         vertexConflicts.emplace_back(make_tuple(downID, upID, coordinate(upLine.front().x + 2, upLine.front().y), coordinate()));
                // }
        //     }
        //     else if(dot_res == -1)
        //         node->sideConflicts_.emplace_back(make_tuple(upID, downID, false));
        // }
        }
    }
}

void ECBS::find_vertexConflict(const Path& iLine, const Path& jLine,
                               int i_ori, int j_ori,
                               int iID, int jID,
                               ECBSNode* node)
{
    int len = iLine.size();
    Path horizonLine, vertiLine;
    int horizonOri, vertiOri, horizonID, vertiID;
    if(iLine.front().x == iLine.back().x)
    {
        horizonLine = iLine;
        vertiLine = jLine;
        horizonOri = i_ori;
        vertiOri = j_ori;
        horizonID = iID;
        vertiID = jID;
    }
    else
    {
        horizonLine = jLine;
        vertiLine = iLine;
        horizonOri = j_ori;
        vertiOri = i_ori;
        horizonID = jID; 
        vertiID = iID;
    }
    bool horizonDri = horizonLine.back().y - horizonLine.front().y > 0 ? true : false;//true->→，false->←
    bool vertiDri = vertiLine.back().x - vertiLine.front().x > 0 ? true : false;// true->↓， false->↑

    coordinate inter_point = getIntersectionPoint(horizonLine, vertiLine);
    if(!(inter_point == coordinate()))
    {
        if(horizonLine.front() == inter_point && horizonLine.front() == vertiLine.back() ||//2条线头尾相交则直接不需要判断
        vertiLine.front() == inter_point && vertiLine.front() == horizonLine.back()) return;

        bool vertFrontisTurn = false, horizonFrontisTurn = false;//判断起点是否为转角点
        for(const auto& turn : node->turnPoints.at(vertiID))
        {
            if(vertiLine.front() == turn)
            {
                vertFrontisTurn == true;
                break;
            }
        }
        for(const auto& turn : node->turnPoints.at(horizonID))
        {
            if(horizonLine.front() == turn)
            {
                vertFrontisTurn == true;
                break;
            }
        }

        if(vertFrontisTurn == horizonFrontisTurn)//考虑了2者都为拐点或者2者都不为拐点
        {
            pair<double,double> horizonTime = oi.GetTime(horizonLine.front(), inter_point);//获取横向到碰撞点
            if(horizonOri == vertiOri)
            {
                vector<pair<double,double>> vertTimes(3, pair<double,double>{-1,-1});
                if(isInLine(vertiLine, coordinate(inter_point.x - 1, inter_point.y)))
                    vertTimes.at(0) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x - 1, inter_point.y));
                vertTimes.at(1) = oi.GetTime(vertiLine.front(), inter_point);
                if(isInLine(vertiLine, coordinate(inter_point.x + 1, inter_point.y)))
                    vertTimes.at(2) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x + 1, inter_point.y));
                for(int i = 0; i < 3; ++i)
                {//思路转换，先按照正常的进行记录，之后到真正去派发的时候再自我验证一下
                    if(common.isTimeOverlap(vertTimes.at(i), horizonTime))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y-1),coordinate(inter_point.x-2,inter_point.y)));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y+1),coordinate(inter_point.x-2,inter_point.y)));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y-1),coordinate(inter_point.x+2,inter_point.y)));
                        else if(!horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y+1),coordinate(inter_point.x+2,inter_point.y)));
                        return;
                    }
                    return;
                }
            }
            else if(horizonOri == 1)
            {
                vector<pair<double,double>> vertTimes(4, pair<double,double>{-1,-1});
                if(isInLine(vertiLine, coordinate(inter_point.x-1, inter_point.y)))
                    vertTimes.at(0) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x-1,inter_point.y));
                vertTimes.at(1) = oi.GetTime(vertiLine.front(), inter_point);
                if(isInLine(vertiLine, coordinate(inter_point.x+1, inter_point.y)))
                    vertTimes.at(2) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x+1,inter_point.y));
                if(isInLine(vertiLine, coordinate(inter_point.x+2, inter_point.y)))
                    vertTimes.at(3) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x+2,inter_point.y));
                for(int i = 0; i < 4; ++i)
                {
                    if(common.isTimeOverlap(vertTimes.at(i), horizonTime))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y-1),coordinate(inter_point.x-2,inter_point.y)));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y+1),coordinate(inter_point.x-2,inter_point.y)));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y-1),coordinate(inter_point.x+3,inter_point.y)));
                        else if(!horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y+1),coordinate(inter_point.x+3,inter_point.y)));
                        return;
                    }
                    return;
                }
            }
            else if(vertiOri == 1)
            {
                vector<pair<double,double>> vertTimes(4, pair<double,double>{-1,-1});
                if(isInLine(vertiLine, coordinate(inter_point.x-2, inter_point.y)))
                    vertTimes.at(0) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x-1,inter_point.y));
                if(isInLine(vertiLine, coordinate(inter_point.x-1, inter_point.y)))
                    vertTimes.at(1) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x+1,inter_point.y));
                vertTimes.at(2) = oi.GetTime(vertiLine.front(), inter_point);
                if(isInLine(vertiLine, coordinate(inter_point.x+1, inter_point.y)))
                    vertTimes.at(3) = oi.GetTime(vertiLine.front(), coordinate(inter_point.x+2,inter_point.y));
                for(int i = 0; i < 4; ++i)
                {
                    if(common.isTimeOverlap(vertTimes.at(i), horizonTime))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y-1),coordinate(inter_point.x-3,inter_point.y)));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y+1),coordinate(inter_point.x-3,inter_point.y)));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y-1),coordinate(inter_point.x+2,inter_point.y)));
                        else if(!horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(inter_point.x,inter_point.y+1),coordinate(inter_point.x+2,inter_point.y)));
                        return;
                    }
                    return;
                }
            }
        }
    }
    else//考虑无重合但是依旧会可能会碰撞的情况
    {
        /*       |  min             ——————————————
                 |
                 |  max       &&          |  min
                                          |
            ___________                   |  max
        */
        if((vertiLine.front().y >= horizonLine.front().y && vertiLine.front().y <= horizonLine.back().y) || 
           (vertiLine.front().y >= horizonLine.back().y && vertiLine.front().y <= horizonLine.front().y))
        {
            int vert_min_x = vertiLine.front().x > vertiLine.back().x ? vertiLine.back().x : vertiLine.front().x;
            int vert_max_x = vertiLine.front().x > vertiLine.back().x ? vertiLine.front().x : vertiLine.back().x;
            if(horizonLine.front().x - vert_min_x == -2)//横上， 竖下(间隔1)
            {
                if(vertiOri == 0 && horizonOri == 1)
                {
                    pair<double,double> vert_time_range = oi.GetTime(vertiLine.front(), coordinate(horizonLine.front().x+2, vertiLine.front().y));
                    pair<double,double> hori_time_range = oi.GetTime(horizonLine.front(), coordinate(horizonLine.front().x, vertiLine.front().y));
                    if(common.isTimeOverlap(vert_time_range, hori_time_range))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate()));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate(horizonLine.front().x+3, vertiLine.front().y)));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate()));
                        else
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate(horizonLine.front().x+3, vertiLine.front().y)));
                        return;
                    }
                    return;
                }  
            }
            else if(horizonLine.front().x - vert_max_x == -2)//横下， 竖上（间隔1）
            {
                if(vertiOri == 1 && horizonOri == 0)
                {
                    pair<double,double> vert_time_range = oi.GetTime(vertiLine.front(), coordinate(horizonLine.front().x-2, vertiLine.front().y));//（x',y'）
                    pair<double,double> hori_time_range = oi.GetTime(horizonLine.front(), coordinate(horizonLine.front().x, vertiLine.front().y));//(x,y)
                    if(common.isTimeOverlap(vert_time_range, hori_time_range))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate(horizonLine.front().x-3, vertiLine.front().y)));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate()));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate(horizonLine.front().x-3, vertiLine.front().y)));
                        else
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate()));
                        return;
                    }
                    return;
                }  
            }
            else if(horizonLine.front().x - vert_min_x == -1)//横上， 竖下(间隔0)
            {
                if((vertiOri == horizonOri) || (vertiOri == 1 && horizonOri == 0))
                {
                    pair<double,double> vert_time_range = oi.GetTime(vertiLine.front(), coordinate(horizonLine.front().x+1, vertiLine.front().y));//（x',y'）
                    pair<double,double> hori_time_range = oi.GetTime(horizonLine.front(), coordinate(horizonLine.front().x, vertiLine.front().y));//(x,y)
                    if(common.isTimeOverlap(vert_time_range, hori_time_range))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate()));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate(vertiLine.front().x+2, vertiLine.front().y)));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate()));
                        else
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate(vertiLine.front().x+2, vertiLine.front().y)));
                        return;
                    }
                    return;
                }
                else// vertori = 0, hori = 1
                {
                    pair<double,double> vert_time_range = oi.GetTime(vertiLine.front(), coordinate(horizonLine.front().x+2, vertiLine.front().y));//（x',y'）
                    pair<double,double> hori_time_range = oi.GetTime(horizonLine.front(), coordinate(horizonLine.front().x, vertiLine.front().y));//(x,y)
                    if(common.isTimeOverlap(vert_time_range, hori_time_range))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate()));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate(vertiLine.front().x+3, vertiLine.front().y)));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate()));
                        else
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate(vertiLine.front().x+3, vertiLine.front().y)));
                        return;
                    }
                    return;
                }  
            }
            else if(horizonLine.front().x - vert_max_x == -1)//横下，竖上
            {
                if((vertiOri == horizonOri) || (vertiOri == 0 && horizonOri == 1))
                {
                    pair<double,double> vert_time_range = oi.GetTime(vertiLine.front(), coordinate(horizonLine.front().x-1, vertiLine.front().y));//（x',y'）
                    pair<double,double> hori_time_range = oi.GetTime(horizonLine.front(), coordinate(horizonLine.front().x, vertiLine.front().y));//(x,y)
                    if(common.isTimeOverlap(vert_time_range, hori_time_range))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate(horizonLine.front().x-2, vertiLine.front().y)));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate()));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate(horizonLine.front().x-2, vertiLine.front().y)));
                        else
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate()));
                        return;
                    }
                    return;
                }
                else// vertori = 1, hori = 0
                {
                    pair<double,double> vert_time_range = oi.GetTime(vertiLine.front(), coordinate(horizonLine.front().x-2, vertiLine.front().y));//（x',y'）
                    pair<double,double> hori_time_range = oi.GetTime(horizonLine.front(), coordinate(horizonLine.front().x, vertiLine.front().y));//(x,y)
                    if(common.isTimeOverlap(vert_time_range, hori_time_range))
                    {
                        if(horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate(horizonLine.front().x-3, vertiLine.front().y)));
                        else if(horizonDri && !vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y-1), coordinate()));
                        else if(!horizonDri && vertiDri)
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate(horizonLine.front().x-3, vertiLine.front().y)));
                        else
                            vertexConflicts.emplace_back(make_tuple(horizonID, vertiID, coordinate(horizonLine.front().x, vertiLine.front().y+1), coordinate()));
                        return;
                    }
                    return;
                }
            }
        }
    }
}

bool ECBS::isInLine(const Path& path, const coordinate& coord)
{
    if((coord.x < 0 || coord.x >= row_) ||
       (coord.y < 0 || coord.y >= col_)) return false;
    for(const auto& c : path)
    {
        if(coord == c) return true;
    }
    return false;
}

void ECBS::resolve_sideConflict(ECBSNode* n1, ECBSNode* n2)
{
    sideConflict conf = n1->sideConflicts_.back();
    n1->sideConflicts_.pop_back();
    n2->sideConflicts_.pop_back();
    if(randomNum_ % 2)
    {
        n1->constraintID_ = conf.first;
        n2->constraintID_ = conf.second;
    }
    else
    {
        n1->constraintID_ = conf.second;
        n2->constraintID_ = conf.first;
    }
}

void ECBS::update_paths(ECBSNode* node)
{
    vector<bool> updated(num_of_agents_, false);
    paths_.resize(num_of_agents_);//少了这个下面加数据一直报错
    while(node != nullptr)
    {
        for(size_t i = 0; i < num_of_agents_; ++i)
        {
            if(!updated.at(i))
            {
                paths_[i] = node->paths_.at(i);
                updated[i] = true;
            }
        }
        node = node->parent_;
    }
}

bool ECBS::generate_child(ECBSNode* node)
{
    if(!find_path(node)) return false;
    else return true;
}

bool ECBS::find_path(ECBSNode* node)
{
    Path path;
    int agvClass = starts_.at(node->constraintID_).second % 2;
    ct_.setCT(node->paths_, node->constraintID_, min_len_);
    Map map = maps_.at(node->constraintID_ % 2);//TODO:为了测试的写法，后面记得改
    path = Astar_.run(map, starts_.at(node->constraintID_).first, ends_.at(node->constraintID_), ct_, agvClass);
    //TODO：这里记录path_cost，但是要删除之前的值
    node->f_val_ = node->f_val_ - node->path_cost_.at(node->constraintID_) + Astar_.path_cost;
    node->path_cost_.at(node->constraintID_) = Astar_.path_cost;
    if(path.empty()) return false;
    node->paths_.at(node->constraintID_) = path;
    return true;
}

void ECBS::resolveVertexConf(ECBSNode* node, vector<bool> updated)
{
    /*
    1、对异常信息处理（其中1点异常、其中2点异常）异常包括：非法点位 / 不在正常路径中
    2、正常情况点信息处理，按随机数进行等待
    3、对重复碰撞的路径点进行考虑，如果 a-b, a-c 都冲突，且已经限制了a,那么依旧限制a，并且取最短的限制段
    */
    int randomNum = randomNum_ % 2;
    int vertexNums = vertexConflicts.size();
    if(vertexNums == 0) return;
    unordered_map<int, vector<coordinate>> usedCoord;
    for(int i = 0; i < vertexNums; ++i)
    {
        vertexConflict& vconf = vertexConflicts.at(i);
        coordinate coord0 = get<2>(vconf);
        coordinate coord1 = get<3>(vconf);
        if(coord0 == coordinate() && coord1 == coordinate()) continue;
        if(coord0 == coordinate())
        {
            int agvID = get<1>(vconf);
            if(validCoord(coord1, agvID))
                usedCoord[agvID].push_back(coord1);
            else
                updated.at(get<1>(vconf)) = true;
        }
        else if(coord1 == coordinate())
        {
            int agvID = get<0>(vconf);
            if(validCoord(coord0, agvID))
                usedCoord[agvID].push_back(coord0);
            else 
                updated.at(get<0>(vconf)) = true;
        }
        else
        {
            int agv0ID = get<0>(vconf);
            int agv1ID = get<1>(vconf);
            if(!validCoord(coord0,agv0ID) && !validCoord(coord1,agv1ID))
                updated.at(get<1>(vconf)) = true;
            if(!validCoord(coord0,agv0ID) && validCoord(coord1,agv1ID))
                usedCoord[agv1ID].push_back(coord1);
            else if(validCoord(coord0,agv0ID) && !validCoord(coord1,agv1ID))
                usedCoord[agv1ID].push_back(coord0);
            else if(validCoord(coord0,agv0ID) && validCoord(coord1,agv1ID))
            {
                if(randomNum)
                    usedCoord[agv0ID].push_back(coord0);
                else
                    usedCoord[agv1ID].push_back(coord1);
            }
        }
    }
    for(auto it = usedCoord.begin(); it != usedCoord.end(); ++it)
    {
        int ID = it->first;
        vector<coordinate> coords = it->second;
        coordinate minlenCoord = coords.at(0);
        coordinate start = paths_.at(ID).front();
        for(coordinate& coord : coords)
        {
            int prvLen = abs(start.x - minlenCoord.x) + abs(start.y - minlenCoord.y);
            int currLen = abs(start.x - coord.x) + abs(start.y - coord.y);
            minlenCoord = prvLen >= currLen ? coord : minlenCoord;
        }
        Path p = paths_.at(ID);
        for(coordinate& c : p)
        {
            if(c == coords.at(0))
            {
                runPaths_.at(ID).push_back(c);
                break;
            }
            runPaths_.at(ID).push_back(c);
        }
        updated.at(ID) = true;
    }
}

bool ECBS::validCoord(const coordinate& c, const int& ID)
{
    if(c.x >= 0 && c.x < row_ && c.y >= 0 && c.y < col_)
    {
        Path& p = paths_.at(ID);
        for(const auto& coord : p)
            if(coord == c) return true;
    }
    return false;
}

vector<Path> ECBS::get_runpaths()
{
    return runPaths_;
}


                                
                                




































































