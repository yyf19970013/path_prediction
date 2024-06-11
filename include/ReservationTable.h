#include "Common.h"

class ReservationTable{

public:
    size_t map_size_;
    int num_of_agents_;

private:
    unordered_map<size_t, list<pair<double,double>>> ct;//针对路径和约束的时间限制表
    vector<vector<bool>> cat;//地方是否可行进标志
    
};
