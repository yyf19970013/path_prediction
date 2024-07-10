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
        cerr << "faile to open file: " << filename << endl;
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

vector<Path> Common::readPathFromFile(const string& filename)
{
    vector<Path> paths;
    Path p;
    ifstream file(filename);
    string line;
    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return paths;
    }
    while (getline(file, line)) {
        if (line.find('#') != string::npos) {
            if (!p.empty()) {
                paths.push_back(p);
                p.clear();
            }
            continue; 
        }

        stringstream ss(line);
        string item;
        int x = 0, y = 0;
        if (getline(ss, item, ',')) {
            x = stoi(item);
        }
        if (getline(ss, item, ',')) {
            y = stoi(item);
        }
        p.emplace_back(coordinate(x, y));
    }
    if (!p.empty()) {
        paths.push_back(p);
    }
    file.close();
    return paths;
}

void Common::Pause()
{
    char ch;
    cout << "Press space key to continue...";
    while (true) {
        std::cin.get(ch);
        if (ch == ' ') {
            break;
        }
        // 清除输入缓冲区
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}