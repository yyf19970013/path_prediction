#include "ConstraintTable.h"

void ConstraintTable::initialize(int row, int col)
{
    row_ = row;
    col_ = col;
    ReverseCT_.resize(row);
    PostiveCT_.resize(row);
    for(int i = 0; i < col; ++i)
    {
        ReverseCT_.at(i).resize(col);
        PostiveCT_.at(i).resize(col);
    }
}

void ConstraintTable::setCT(const vector<Path>& paths, int replanID, int len)
{
    vector<Path> lenPaths(paths.size());
    int replanAgvClass = starts_.at(replanID).second % 2;
    for(int i = 0 ; i < paths.size(); ++i)
    {
        if(paths.at(i).size() <= 6)
        {
            lenPaths[i] = paths.at(i);
            continue;
        } 
        for(int j = 0; j < len; ++j)
            lenPaths.at(i).push_back(paths.at(i).at(j));
    }
    for(int i = 0; i < starts_.size(); ++i)
    {
        if(replanID == i) continue;
        int agvClass = starts_.at(i).second % 2;
        if(replanAgvClass)
            setPostiveCT(lenPaths.at(i), agvClass);
        else
            setReverseCT(lenPaths.at(i), agvClass);
    }
}

void ConstraintTable::setReverseCT(const Path& lenPath, int agvClass)
{
    if(agvClass)//正
    {
        for(const auto& c : lenPath)
        {
            pair<double,double> t = oi.GetTime(lenPath.front(), c);
            if(vaildCoord(coordinate(c.x-1,c.y))) ReverseCT_.at(c.x-1).at(c.y).push_back(t);
            if(vaildCoord(c)) ReverseCT_.at(c.x).at(c.y).push_back(t);
            if(vaildCoord(coordinate(c.x+1,c.y))) ReverseCT_.at(c.x+1).at(c.y).push_back(t);
            if(vaildCoord(coordinate(c.x+2,c.y))) ReverseCT_.at(c.x+2).at(c.y).push_back(t);
        }
    }
    else//反
    {
        for(const auto& c : lenPath)
        {
            pair<double,double> t = oi.GetTime(lenPath.front(), c);
            if(vaildCoord(coordinate(c.x-1,c.y))) ReverseCT_.at(c.x-1).at(c.y).push_back(t);
            if(vaildCoord(c)) ReverseCT_.at(c.x).at(c.y).push_back(t);
            if(vaildCoord(coordinate(c.x+1,c.y))) ReverseCT_.at(c.x+1).at(c.y).push_back(t);
        }
    }  
}

void ConstraintTable::setPostiveCT(const Path& lenPath, int agvClass)
{
    if(!agvClass)//反
    {
        for(const auto& c : lenPath)
        {
            pair<double,double> t = oi.GetTime(lenPath.front(), c);
            if(vaildCoord(coordinate(c.x-2,c.y))) PostiveCT_.at(c.x-2).at(c.y).push_back(t);
            if(vaildCoord(coordinate(c.x-1,c.y))) PostiveCT_.at(c.x-1).at(c.y).push_back(t);
            if(vaildCoord(c)) PostiveCT_.at(c.x).at(c.y).push_back(t);
            if(vaildCoord(coordinate(c.x+1,c.y))) PostiveCT_.at(c.x+1).at(c.y).push_back(t);
        }
    }
    else if(agvClass)//正
    {
        for(const auto& c : lenPath)
        {
            pair<double,double> t = oi.GetTime(lenPath.front(), c);
            if(vaildCoord(coordinate(c.x-1,c.y))) PostiveCT_.at(c.x-1).at(c.y).push_back(t);
            if(vaildCoord(c)) PostiveCT_.at(c.x).at(c.y).push_back(t);
            if(vaildCoord(coordinate(c.x+1,c.y))) PostiveCT_.at(c.x+1).at(c.y).push_back(t);
        }
    }
}

bool ConstraintTable::vaildCoord(const coordinate& coord)
{
    if(coord.x < 0 || coord.x >= row_ || coord.y < 0 || coord.y >= col_) return false;
    return true;
}











