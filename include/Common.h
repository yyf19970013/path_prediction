#pragma once

#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     
#include <iomanip>      
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "OutIn.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_set;
using boost::unordered_map;

using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
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

enum Conf{Vertex, Side};

typedef tuple<coordinate, int, int> Constraint;//[0]:位置，[1]:最早到达时间， [2]:最晚到达时间//TODO:后续还需再琢磨
typedef tuple<int, int, int, int, Conf> Conflict;//[0]:AGV1、[1]:AGV2、[2]:碰撞起点、[3]:碰撞终点、[4]：碰撞类型

typedef tuple<int, int, int, int> mapRoute;//顺时针（上、右、下、左）1:不可走，0：可行驶
typedef vector<vector<pair<int,mapRoute>>> map;

typedef pair<int,int> coordinate;//单坐标点

typedef vector<coordinate> Path;//所有路径点

typedef vector<pair<coordinate,coordinate>> sidePath;//只记录段信息,且左闭右开（从当前段开头，到当前段最后一个点（下一段的起点）前）


class Common{

public:
    int getRow(){return row;}
    int getCol(){return col;}

private:

    int row = 20;
    int col = 55;

};