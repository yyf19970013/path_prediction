#ifndef SOLVER_H
#define SOLVER_H

#include "Common.h"
#include "OutIn.h"
#include "ECBS.h"
//此类负责更新地图，最外层的解决方案

class Solver{

public:

    Solver(vector<Map>& maps, vector<pair<coordinate, int>>& starts, vector<coordinate>& ends, int min_len, double turnstime);
    ~Solver() = default;
    void initialInfo();
    void run();
    void updateStarts();
    void updateEnds(vector<coordinate> ends);
    string serializePath(const Path& paths);
    string serializePaths(const vector<Path>& paths);
    //规划要考虑最后返给一个再调取的时间

    vector<pair<coordinate, int>> starts_;//int: 1->正车无货， 2->反车无货， 3->正车有货， 4->反车有货
    vector<coordinate> ends_;

    vector<Map> maps_;

    OutIn oi_;
    ECBS ECBS_;
    Common common_;

    int min_len_;
    double turntime_;
    

    vector<vector<Path>> pathPool;
    

private: 

};

#endif