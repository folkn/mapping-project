#include <iostream>
#include "src/lib/trojanmap.h"
#include <math.h>
#include <iomanip>

int main()
{

  TrojanMap x;
  x.CreateGraphFromCSVFile();

  // DEBUG : Print (map) data
  // std::map<std::string, Node> data = x.data_out();
  // for (auto elem : data)
  // {
  //   if (elem.second.name != "") //show elements with names only.
  //   {
  //     std::cout << std::endl
  //               << elem.second.id << "\t" << elem.second.name[0] << "\t" << elem.second.lat << "," << elem.second.lon <<"\tNeighbors";
  //     for (auto neighbor : elem.second.neighbors) {std::cout << neighbor<<",";}
  //   }
  // }

  // DEBUG : Get Lat
  // std::cout << x.GetLat("855440981");

  // DEBUG: Total Distance
  // std::vector<std::string> path {"123120189","1931345270", "4011837224","4011837229","2514542032"};//,"2514541020","6807909279","63068532","214470792","4015477529","123120189"};
  // std::cout << "Path distance "<< std::setprecision(200) << x.CalculatePathLength(path) << std::endl;
  // x.TravellingTrojan(path);
  // std::cout<<"nbr "<<data["123120189"].neighbors[0] <<std::endl;
  
// x.CalculateShortestPath("Crosswalk2", "61");
  // x.PlotPath(path);

   x.PrintMenu();
  return 0;
}