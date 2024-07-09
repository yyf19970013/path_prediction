#ifndef COMMON_H
#define COMMON_H

#include <algorithm>
#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     
#include <iomanip>
#include <random>
#include <chrono>
#include <string>
#include <sstream>      
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>


using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_set;
using boost::unordered_map;

using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;

//using namespace std; //TODO:如果这里定义了统一std，会导致common中自定义的map意义不明确报错
using std::sort;
using std::swap;
using std::vector;
using std::list;
using std::set;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;
using std::random_device;
using std::ostringstream;
using std::ifstream;
using std::stringstream;

// enum ConfClass{Vertex, Side};
 
struct coordinate{//TODO：如果需要将其作为一个类型传递进函数中，需要保证参数名字和struct名字不能相同，不然存在歧义，导致编译有问题
    int x;
    int y;
    coordinate():x(-1),y(-1){}
    coordinate(int x_, int y_):x(x_),y(y_){}
    bool operator==(const coordinate& other) const
    {
        return x == other.x && y == other.y;
    }
    bool operator!=(const coordinate& other) const
    {
        return x != other.x && y != other.y;
    }
};
//------------------碰撞相关部分-------------------//
typedef vector<pair<double, double>> pointConstraint;//pair.first:最早到达时间；pair.second:最晚到达时间（vector包含当前点存在的多条路径影响下存在的多个时间限制）
typedef pair<int, int> sideConflict;//[0]:AGV1，[1]:AGV2，[2]:true->x, false->y(沿x或y)//TODO:这个[2]的信息感觉没啥意思
typedef tuple<int, int, coordinate, coordinate> vertexConflict;//[0]:AGV1，[1]:AGV2，[2]:AGV1等待点，[3]:AGV2等待点
//-------------------地图部分---------------------//
typedef tuple<bool, bool, bool, bool> mapRoute;//当前格可行驶方向(出的方向)，顺时针（上、右、下、左）,true：可走，false：阻挡
typedef vector<vector<mapRoute>> Map;
//-------------------路径部分---------------------//
typedef vector<coordinate> Path;//所有路径点
typedef vector<Path> sidePath;//TODO:只记录段信息,且左闭右开（从当前段开头，到当前段最后一个点（下一段的起点）前）

typedef vector<tuple<int, int, int, int>> rectConstriants;//暂时好像不需要

class Common{
public:
    int getRow(){return row_;}
    int getCol(){return col_;}
    void setRow(int row){row_ = row;}
    void setCol(int col){col_ = col;}
    
    bool isTimeOverlap(pair<double,double> i_range, pair<double,double> j_range);
    Path readCoordFromFile(const string& filename);

private:

    int row_;
    int col_;

};

#endif