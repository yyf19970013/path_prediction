#ifndef CONSTRAINTTABLE_H
#define CONSTRAINTTABLE_H

#include "Common.h"
#include "OutIn.h"

class ConstraintTable{

public:
    ConstraintTable()=default;
    ~ConstraintTable() = default;
    void initialize(int row, int col);
    void setCT(const vector<Path>& paths, int replanID, int len);
    void setReverseCT(const Path& paths, int ID);
    void setPostiveCT(const Path& paths, int ID);
    bool vaildCoord(const coordinate& coord);
    vector<vector<pointConstraint>> getReverseCT(){return ReverseCT_;}
    vector<vector<pointConstraint>> getPostiveCT(){return PostiveCT_;}
    size_t map_size_;
    int num_of_agents_;

    OutIn oi;
    Common common;
    int row_;
    int col_;
    vector<pair<coordinate, int>> starts_;
    vector<vector<pointConstraint>> ReverseCT_;//面向反车
    vector<vector<pointConstraint>> PostiveCT_;//面向正车

private:
    
};

#endif