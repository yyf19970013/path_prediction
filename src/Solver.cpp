#include "Solver.h"

Solver::Solver(const vector<vector<pair<int,int>>>& map,// <是否可行驶，可行驶方向>
       const vector<tuple<int,int,int,int>>& starts,// <ID、x、y、车辆状态(0:正车、1：反车、2:正车有货、3：反车有货)>
       const vector<tuple<int,int,int,int>>& ends){
        starts_ = starts;
        ends_ = ends;
        GetMaps(map);
       }
void Solver::GetMaps(const vector<vector<pair<int,int>>>& map){
    
}
bool Solver::solver(){;}