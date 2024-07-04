#include "ConstraintTable.h"

void ConstraintTable::setCT(const vector<Path>& paths, int replanID, int len)
{
    int agvClass = starts_.at(replanID).second % 2;
    if(agvClass)
        setPostiveCT(paths, replanID, len);
    else
        setPostiveCT(paths, replanID, len);
}

void ConstraintTable::setReverseCT(const vector<Path>& paths, int replanID ,int len)
{
    vector<Path> lenPaths(paths.size());
    for(int i = 0 ; i < paths.size(); ++i)
    {
        for(int j = 0; j < len; ++i)
            lenPaths.at(i).push_back(paths.at(i).at(j));
    }

    for(int i = 0; i < paths.size(); ++i)
    {
        if(i == replanID) continue;
        // bool x_dri = lenPaths.at(i).front().x == lenPaths.at(i).back().x ? true : false;
        if(starts_.at(i).second % 2)//沿x且正车
        {
            Path ipath = lenPaths.at(i);
            for(const auto& c : ipath)
            {
                pair<double,double> t = oi.GetTime(ipath.front(), c);
                if(vaildCoord(coordinate(c.x-1,c.y))) ReverseCT_.at(c.x-1).at(c.y).push_back(t);
                if(vaildCoord(c)) ReverseCT_.at(c.x).at(c.y).push_back(t);
                if(vaildCoord(coordinate(c.x+1,c.y))) ReverseCT_.at(c.x+1).at(c.y).push_back(t);
                if(vaildCoord(coordinate(c.x+2,c.y))) ReverseCT_.at(c.x+2).at(c.y).push_back(t);
            }
        }
        else if(!starts_.at(i).second % 2)//沿x且反车
        {
            Path ipath = lenPaths.at(i);
            for(const auto& c : ipath)
            {
                pair<double,double> t = oi.GetTime(ipath.front(), c);
                if(vaildCoord(coordinate(c.x-1,c.y))) ReverseCT_.at(c.x-1).at(c.y).push_back(t);
                if(vaildCoord(c)) ReverseCT_.at(c.x).at(c.y).push_back(t);
                if(vaildCoord(coordinate(c.x+1,c.y))) ReverseCT_.at(c.x+1).at(c.y).push_back(t);
            }
        }
    }
}

void ConstraintTable::setPostiveCT(const vector<Path>& paths, int replanID ,int len)
{
    vector<Path> lenPaths(paths.size());
    for(int i = 0 ; i < paths.size(); ++i)
    {
        for(int j = 0; j < len; ++i)
            lenPaths.at(i).push_back(paths.at(i).at(j));
    }

    for(int i = 0; i < paths.size(); ++i)
    {
        if(i == replanID) continue;
        // bool x_dri = lenPaths.at(i).front().x == lenPaths.at(i).back().x ? true : false;
        if(!starts_.at(i).second % 2)
        {
            Path ipath = lenPaths.at(i);
            for(const auto& c : ipath)
            {
                pair<double,double> t = oi.GetTime(ipath.front(), c);
                if(vaildCoord(coordinate(c.x-1,c.y))) PostiveCT_.at(c.x-1).at(c.y).push_back(t);
                if(vaildCoord(c)) PostiveCT_.at(c.x).at(c.y).push_back(t);
                if(vaildCoord(coordinate(c.x+1,c.y))) PostiveCT_.at(c.x+1).at(c.y).push_back(t);
                if(vaildCoord(coordinate(c.x+2,c.y))) PostiveCT_.at(c.x+2).at(c.y).push_back(t);
            }
        }
        else if(starts_.at(i).second % 2)
        {
            Path ipath = lenPaths.at(i);
            for(const auto& c : ipath)
            {
                pair<double,double> t = oi.GetTime(ipath.front(), c);
                if(vaildCoord(coordinate(c.x-1,c.y))) PostiveCT_.at(c.x-1).at(c.y).push_back(t);
                if(vaildCoord(c)) PostiveCT_.at(c.x).at(c.y).push_back(t);
                if(vaildCoord(coordinate(c.x+1,c.y))) PostiveCT_.at(c.x+1).at(c.y).push_back(t);
            }
        }
    }
}

bool ConstraintTable::vaildCoord(const coordinate& coord)
{
    int row = common.getRow();
    int col = common.getCol();
    if(coord.x < 0 || coord.x >= row || coord.y < 0 || coord.y >= col) return false;
    return true;
}











