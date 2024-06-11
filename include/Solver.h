#pragma once
#include "Common.h"
//此类负责更新地图，最外层的解决方案

class Solver{

public:

    Solver(const vector<vector<pair<int,int>>>& map,// <是否可行驶，可行驶方向>
           const vector<tuple<int,int,int,int>>& starts,// <ID、x、y、车辆状态(0:正车、1：反车、2:正车有货、3：反车有货)>
           const vector<tuple<int,int,int,int>>& ends);// <ID、x、y> 需要添加一个是否存在任务的状态
    ~Solver() = default;
    void GetMaps(const vector<vector<pair<int,int>>>& map);
    bool solver();
    //规划要考虑最后返给一个再调取的时间
private: 
    vector<tuple<int,int,int,int>> starts_;
    vector<tuple<int,int,int,int>> ends_;

    unordered_map<int,map> maps_;

    int agentNum_;
    double timeLimit_;

};