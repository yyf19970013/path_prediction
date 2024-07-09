#include "OutIn.h"

pair<double,double> OutIn::GetTime(const coordinate& st, const coordinate& end)
{
   double time = abs(st.x - end.x) + abs(st.y - end.y);
   return pair<double,double>{time, time + 2.0};
}