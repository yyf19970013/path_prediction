#ifndef OUTIN_H
#define OUTIN_H
#include "Common.h"

class OutIn
{
public:
    OutIn() = default;
    ~OutIn() = default;

    bool HasV(int ID);
    pair<double,double> GetTime(const coordinate& st, const coordinate& end);

    // vector<Map> getMaps();
    // vector<pair<coordinate, int>> getStarts();
    // vector<coordinate> getEnds();
    // bool arriveEnd(int agvID);
    // coordinate getCoord(int agvID);
    // coordinate getRealtimeCoord(int agvID);

private: 

};

#endif