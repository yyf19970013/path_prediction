#include "Solver.h"

Solver::Solver(vector<Map>& maps, vector<pair<coordinate, int>>& starts, vector<coordinate>& ends, int min_len, double turntime)
{
    maps_ = maps;
    starts_ = starts;
    ends_ = ends;
    min_len_ = min_len;
    turntime_ = turntime;
    ocbs_.initialize(maps,starts,ends,min_len,turntime);
}

void Solver::run()
{
    cout << "run1" << endl;
    // int times = 1;
    while(1)
    {
        // if (i == 0)
        //     starts_ = oi_.getStarts();
        // ends_ = oi_.getEnds();
        // updateStarts();
        // updateEnds(ends_);
        bool succ = ocbs_.run();
        // vector<Path> runpath = ocbs_.get_runpaths();
        // for(int i = 0; i < starts_.size(); ++i)
        // {
        //     if(pathPool.at(i).size() < 2)  // 报错
        //     {
        //         pathPool.at(i).at(0) = pathPool.at(i).at(1);
        //         pathPool.at(i).push_back(ocbs_.runPaths_.at(i));
        //     }
        // }
        // string res = serializePaths(runpath);
        // cout << res << endl;
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
                starts_.at(i).first = ocbs_.runPaths_.front().back();
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
            for(size_t j = 0; i < ocbs_.spaths_.size(); ++i)
            {
                for(const auto& p : ocbs_.spaths_.at(j))
                {
                    if(p.front() == ends.at(i)) ends_.at(i) = ends.at(i);
                }
            }
        }
    }
}

string Solver::serializePaths(const vector<Path>& paths)
{
    ostringstream oss;
    for (size_t i = 0; i < paths.size(); ++i) {
        oss << serializePath(paths[i]);
        if (i != paths.size() - 1) {
            oss << "_";
        }
    }
    return oss.str();
}

string Solver::serializePath(const Path& path) {
    ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i) {
        oss << "(" << path[i].x << "," << path[i].y << ")";
        if (i != path.size() - 1) {
            oss << "->";
        }
    }
    return oss.str();
}
