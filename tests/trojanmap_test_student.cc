#include "src/lib/trojanmap.h"

#include <map>
#include <vector>

#include "gtest/gtest.h"

TEST(TrojanMapTest, AUTOCOMPLETE)
{
  TrojanMap m;
  m.CreateGraphFromCSVFile();

  // Test the simple case
  auto names = m.Autocomplete("Ch");
  std::vector<std::string> gt1 = {"ChickfilA", "Chipotle Mexican Grill"}; // groundtruth for "Ch"
  EXPECT_EQ(names, gt1);
  // Test the lower case
  names = m.Autocomplete("ch");
  std::vector<std::string> gt2 = {"ChickfilA", "Chipotle Mexican Grill"}; // groundtruth for "ch"
  EXPECT_EQ(names, gt2);
  // Test the lower and upper case
  names = m.Autocomplete("cH");
  std::vector<std::string> gt3 = {"ChickfilA", "Chipotle Mexican Grill"}; // groundtruth for "cH"
  EXPECT_EQ(names, gt3);
}

TEST(TrojanMapTest, GET_LATITUDE_LONGITUDE_FROM_ID)
{
  TrojanMap x;
  x.CreateGraphFromCSVFile();

  //Astor Motel
  EXPECT_EQ(x.GetLat("855440981"), 34.024536);
  EXPECT_EQ(x.GetLon("855440981"), -118.2755802);

  //Parking Center
  EXPECT_EQ(x.GetLat("732642214"), 34.0199998);
  EXPECT_EQ(x.GetLon("732642214"), -118.277186);
  //UNKNOWN
  EXPECT_EQ(x.GetLat("599599599599"), -1);
  EXPECT_EQ(x.GetLon("599599599599"), -1);
}

TEST(TrojanMapTest, GET_NAME_FROM_ID)
{
  TrojanMap x;
  x.CreateGraphFromCSVFile();
  //Astor Motel
  EXPECT_EQ(x.GetName("855440981"), "Astor Motel");
  //Parking Center
  EXPECT_EQ(x.GetName("732642214"), "Parking Center");
  //Unknown
  EXPECT_EQ(x.GetName("599599599599"), "");
}

TEST(TrojanMapTest, GET_NEIGHBORS_FROM_ID)
{
  TrojanMap x;
  x.CreateGraphFromCSVFile();

  //Astor Motel
  std::vector<std::string> astor{"269636455"};
  EXPECT_EQ(x.GetNeighborIDs("855440981"), astor);
  //Parking Center
  std::vector<std::string> parking{"4012759746"};
  EXPECT_EQ(x.GetNeighborIDs("732642214"), parking);
  //id = 932378219
  std::vector<std::string> idid{"6226313827", "6814990112", "6226313826"};
  EXPECT_EQ(x.GetNeighborIDs("932378219"), idid);
  //Unknown
  std::vector<std::string> unkn{};
  EXPECT_EQ(x.GetNeighborIDs("599599599599"), unkn);
}

TEST(TrojanMapTest, GET_POSITION_FROM_NAME)
{
  TrojanMap x;
  x.CreateGraphFromCSVFile();
  //std::pair<double, double> latlon (GetLat(elem.second.id), GetLon(elem.second.id))
  //Astor Motel
  std::pair<double, double> astor(34.024536, -118.2755802);
  EXPECT_EQ(x.GetPosition("Astor Motel"), astor);
  //Parking Center
  std::pair<double, double> parking(34.0199998, -118.277186);
  EXPECT_EQ(x.GetPosition("Parking Center"), parking);
  //Unknown name
  std::pair<double, double> unkn(-1, -1);
  EXPECT_EQ(x.GetPosition("OurArash"), unkn);
}

TEST(TrojanMapTest, CALC_PATH_LENGTH)
{
  // Checking CalculateDistance and CalculateDistance, and related functions
  // Checks whether the calculated path length differs less than 10^-5 miles from the actual distance.
  // Not checking for exact equality to take account for compiler precision and roundings.
  TrojanMap x;
  x.CreateGraphFromCSVFile();
  std::vector<std::string> path{"123120189", "1931345270", "4011837224", "4011837229", "2514542032", "2514541020", "6807909279", "63068532", "214470792", "4015477529", "123120189"};
  bool error = (x.CalculatePathLength(path) - 4.61742181525805239772353161242790520191192626953125) >= 0.00001;
  EXPECT_EQ(error, false);
}

TEST(TrojanMapTest, TSP_Test_Brute_Force) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1873056015", "213332060", "1931345270"}; // Input location ids 
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1873056015", "213332060", "1931345270", "1873056015"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = true;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest,TwoOpt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1862312636", "7424270441", "67666219", "4015405548", "4015203110", "6807439002"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"1862312636", "7424270441", "67666219", "4015405548", "4015203110", "6807439002","1862312636"}; // benchmark, 2 opt should do better than this order, which is the original order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt >= result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt >= result.second[result.second.size()-1]) // counterclockwise //we need results.second to be less that GT for both conditions
    flag = true;
  
  EXPECT_EQ(flag, true);
}