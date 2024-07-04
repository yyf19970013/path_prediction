#include "Solver.h"

Solver::Solver(vector<Map>& maps, vector<pair<coordinate, int>>& starts, vector<coordinate>& ends, int min_len, double turntime)
{
    maps_ = maps;
    starts_ = starts;
    ends_ = ends;
    min_len_ = min_len;
    turntime_ = turntime;
    ECBS_.initialize(maps,starts,ends,min_len,turntime);
}


void Solver::run()
{
    while(1)
    {
        // if (i == 0)
        //     starts_ = oi_.getStarts();
        // ends_ = oi_.getEnds();
        // updateStarts();
        // updateEnds(ends_);
        ECBS_.run();
        vector<Path> runpath = ECBS_.get_runpaths();
        // for(int i = 0; i < starts_.size(); ++i)
        // {
        //     if(pathPool.at(i).size() < 2)
        //     {
        //         pathPool.at(i).at(0) = pathPool.at(i).at(1);
        //         pathPool.at(i).push_back(ECBS_.runPaths_.at(i));
        //     }
        // }
        cout << "OK" << endl;
    }
}

void Solver::updateStarts()
{
    for(int i = 0; i < starts_.size(); ++i)
    {
        if(pathPool.at(i).empty())//需要修改
        {
            // if(oi_.arriveEnd(i))
            //     starts_.at(i).first = oi_.getCoord(i);
            // else
                starts_.at(i).first = ECBS_.runPaths_.front().back();
        }
    }
}

void Solver::updateEnds(vector<coordinate> ends)
{
    for(size_t i = 0; i < ends.size(); ++i)
    {
        if(ends_.at(i) == ends.at(i)) continue;
        else if(ends_.at(i) == coordinate()) ends_.at(i) = ends.at(i);
        else if(!(ends.at(i) == coordinate()) && !(ends_.at(i) == coordinate()))
        {
            for(size_t j = 0; i < ECBS_.spaths_.size(); ++i)
            {
                for(const auto& p : ECBS_.spaths_.at(j))
                {
                    if(p.front() == ends.at(i)) ends_.at(i) = ends.at(i);
                }
            }
        }
    }
}

