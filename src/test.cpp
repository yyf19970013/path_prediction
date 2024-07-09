#include "Solver.h"

int main()
{
  int row = 20, col = 20;
  vector<vector<int>> init_map{//20 * 20
  // 0 1 2 3 4 5 6 7 8 910111213141516171819
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//0
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//1
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//2
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//3
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//5
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//6
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//7
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//8
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//9
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},//10
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//11
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//12
    {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},//13
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//14
    {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//15
    {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},//16
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//17
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//18
    {0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//19
  };
    vector<vector<int>> init_map1{//20 * 20
  // 0 1 2 3 4 5 6 7 8 910111213141516171819
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//0
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//1
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//2
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//3
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//5
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//6
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//7
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//8
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},//9
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},//10
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//11
    {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},//12
    {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},//13
    {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//14
    {0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},//15
    {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},//16
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//17
    {0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//18
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},//19
  };

  vector<vector<int>> init_map2{//20 * 20
  // 0 1 2 3 4 5 6 7 8 910111213141516171819
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},//0
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//1
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//2
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//3
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//4
    {0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//5
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//6
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//7
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//8
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},//9
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},//10
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},//11
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//12
    {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},//13
    {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},//14
    {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//15
    {0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},//16
    {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0},//17
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//18
    {0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//19
  };

  
  vector<Map> maps(4, Map(20, vector<mapRoute>(20,mapRoute(true,true,true,true))));
  for(int i = 0; i < 2; ++i)
  {
    Map map(20, vector<mapRoute>(20, tuple<bool,bool,bool,bool>{true,true,true,true}));
    if(i == 0)
    {
      for(int i = 0; i < 20; ++i)
      {
        for(int j = 0; j < 20; ++j)
        {
          if(init_map1.at(i).at(j) == 1)
          {
            get<0>(map.at(i).at(j)) = false;
            get<1>(map.at(i).at(j)) = false;
            get<2>(map.at(i).at(j)) = false;
            get<3>(map.at(i).at(j)) = false;
          }
          else
          {
            if(i-1 < 0) get<0>(map.at(i).at(j)) = false;
            else if(init_map1.at(i-1).at(j) == 1) get<0>(map.at(i).at(j)) = false;
            if(j+1 >= col) get<1>(map.at(i).at(j)) = false;
            else if(init_map1.at(i).at(j+1) == 1) get<1>(map.at(i).at(j)) = false;
            if(i+1 >= row) get<2>(map.at(i).at(j)) = false;
            else if(init_map1.at(i+1).at(j) == 1) get<2>(map.at(i).at(j)) = false;
            if(j-1 < 0) get<3>(map.at(i).at(j)) = false;
            else if(init_map1.at(i).at(j-1) == 1) get<3>(map.at(i).at(j)) = false;
          }
        }
      }
      maps[0] = map;
    }
    else if(i == 1)
    {
      for(int i = 0; i < 20; ++i)
      {
        for(int j = 0; j < 20; ++j)
        {
          if(init_map1.at(i).at(j) == 1)
          {
            get<0>(map.at(i).at(j)) = false;
            get<1>(map.at(i).at(j)) = false;
            get<2>(map.at(i).at(j)) = false;
            get<3>(map.at(i).at(j)) = false;
          }
          else
          {
            if(i-1 < 0) get<0>(map.at(i).at(j)) = false;
            else if(init_map2.at(i-1).at(j) == 1) get<0>(map.at(i).at(j)) = false;
            if(j+1 >= col) get<1>(map.at(i).at(j)) = false;
            else if(init_map2.at(i).at(j+1) == 1) get<1>(map.at(i).at(j)) = false;
            if(i+1 >= row) get<2>(map.at(i).at(j)) = false;
            else if(init_map2.at(i+1).at(j) == 1) get<2>(map.at(i).at(j)) = false;
            if(j-1 < 0) get<3>(map.at(i).at(j)) = false;
            else if(init_map2.at(i).at(j-1) == 1) get<3>(map.at(i).at(j)) = false;
          }
        }
      }
      maps[1] = map;
    }
  }
  maps[2] = maps.at(0);
  maps[3] = maps.at(1);
  cout << "maps1-size" << maps.at(0).size() << " x " << maps.at(0).size() << endl;
  cout << "maps2-size" << maps.at(1).size() << " x " << maps.at(0).size() << endl;
  
  assert(!maps.at(0).empty());
  assert(!maps.at(1).empty());
  assert(!maps.at(2).empty());
  assert(!maps.at(3).empty());
  cout << "Maps OK!" << endl;
  /*
  起点部分
  */
  vector<pair<coordinate,int>> starts;
  starts.emplace_back(make_pair(coordinate(5,7),1));//5 7
  // starts.emplace_back(make_pair(coordinate(6,10),1));
  // starts.emplace_back(make_pair(coordinate(12,4),1));
  // starts.emplace_back(make_pair(coordinate(6,17),1));// 6 17
  // starts.emplace_back(make_pair(coordinate(9,2),2));
  // starts.emplace_back(make_pair(coordinate(18,17),2));
  /*
  终点部分
  */
  vector<coordinate> ends;
  ends.emplace_back(coordinate(17,17));// 17
  // ends.emplace_back(coordinate(16,3));
  // ends.emplace_back(coordinate(1,9));
  // ends.emplace_back(coordinate(12,10));//12 9
  // ends.emplace_back(coordinate(17,10));
  // ends.emplace_back(coordinate(6,7));



  Astar astar;
  ECBS ecbs;
  Common common;
  ConstraintTable ct;

  ecbs.initialize(maps,starts,ends,6,2);
  astar.setMapsize(maps.at(0));
  vector<Path> ps;
  //---------------------路径数据--------------------------------------
  //路径测试对1(交叉)： vertP1_A:正车竖线路径A对 - horiP1_A:反车
  Path vertP_1_A = common.readCoordFromFile("/home/yyf/path_prediction/path/vertical/vert_1_A.txt");
  Path vertP_1_B = common.readCoordFromFile("/home/yyf/path_prediction/path/vertical/vert_1_B.txt");
  Path horiP_1_A = common.readCoordFromFile("/home/yyf/path_prediction/path/horizon/hori_1_A.txt");
  Path horiP_1_B = common.readCoordFromFile("/home/yyf/path_prediction/path/horizon/hori_1_B.txt");
  Path horiP_1_C = common.readCoordFromFile("/home/yyf/path_prediction/path/horizon/hori_1_C.txt");
  
  // ps.push_back(vertP_1_A);
  // ps.push_back(vertP_1_B);
  ps.push_back(horiP_1_A);
  ps.push_back(horiP_1_B);
  ps.push_back(horiP_1_C);

  vector<pair<coordinate,int>> starts2;
  // starts2.emplace_back(make_pair(coordinate(3,6),1));
  // starts2.emplace_back(make_pair(coordinate(9,10),1));
  starts2.emplace_back(make_pair(coordinate(4,8),1));
  starts2.emplace_back(make_pair(coordinate(7,1),1));
  starts2.emplace_back(make_pair(coordinate(8,6),1));
  
  
  //------------------------------------------------------------------
  /*
  TEST1：路径生成测试
  TEST2: 减少路径转弯次数测试
  */
  Path p = astar.run(maps.at(0), starts.at(0).first, ends.front());
  cout << "----------------origin-----------------" << endl;
  astar.drawPath(p, init_map1);
  cout << "--------------minTurnsPath-------------" << endl;
  Path minp = ecbs.minTurnsPath(p, starts.front().second);
  astar.drawPath(minp, init_map1);
  //-------------------------------------------------------------------
  coordinate vA_hA_inner_c = ecbs.getIntersectionPoint(horiP_1_A, vertP_1_A);//获取交并点
  vector<int> vA_vB_inner_p = ecbs.getIntersectionLine(vertP_1_A, vertP_1_B, false);//获取交并线的4个交点(纵向)
  vector<int> hA_hB_inner_p = ecbs.getIntersectionLine(horiP_1_A, horiP_1_B, true);//获取交并线的4个交点(横向)
  
  ct.starts_ = starts2;
  ct.initialize(init_map1.size(),init_map1.size());
  ct.setCT(ps, 1, 6);
  



  while(1);
  return 0;
}