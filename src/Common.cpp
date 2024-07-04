#include "Common.h"

bool Common::isTimeOverlap(pair<double,double> i_range, pair<double,double> j_range)
{
    if(i_range.first > j_range.second || j_range.first > i_range.second) return false;
    return true;
}