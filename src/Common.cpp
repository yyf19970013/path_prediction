#include "Common.h"

bool Common::isTimeOverlap(pair<double,double> i_range, pair<double,double> j_range)
{
    if(i_range.first > j_range.second || j_range.first > i_range.second) return false;
    return true;
}

Path Common::readCoordFromFile(const string& filename)
{
    Path p;
    ifstream file(filename);
    string line;

    if(!file.is_open())
    {
        cerr << "faile to open !" << filename << endl;
    }

    while(getline(file, line))
    {
        stringstream ss(line);
        string item;
        int x, y;
        if (std::getline(ss, item, ','))
        {
            x = std::stoi(item);
        }
        if (std::getline(ss, item, ','))
        {
            y = std::stoi(item);
        }
        p.emplace_back(coordinate(x,y));
    }
    file.close();
    return p;
}