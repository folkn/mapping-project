#include "trojanmap.h"

#include <iomanip>
#include <iostream>

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <utility>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu()
{

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = Autocomplete(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0)
    {
      for (auto x : results)
        std::cout << x << std::endl;
    }
    else
    {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = GetPosition(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1)
    {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    }
    else
    {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                            "
        "      \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto results = CalculateShortestPath(input1, input2);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0)
    {
      for (auto x : results)
        std::cout << x << std::endl;
      PlotPath(results);
    }
    else
    {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl
              << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data)
    {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);

    // Method 1 2-OPT
    std::cout << "Calculating 2OPT..." << std::endl;
    auto results = TravellingTrojan_2opt(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0)
    {
      for (auto x : results.second[results.second.size() - 1])
        std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size() - 1]);
    }
    else
    {
      std::cout << "The size of the path is 0" << std::endl;
    }
    // Method 2 Brute Force
    std::cout << menu << "\n\n"
              << "Calculating BRUTE FORCE..." << std::endl;
    auto results2 = TravellingTrojan(locations);
    menu = "\n*************************Results******************************\n";
    std::cout << menu;
    //CreateAnimation(results2.second);
    if (results2.second.size() != 0)
    {
      for (auto x : results2.second[results2.second.size() - 1])
        std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results2.first << std::endl;
      PlotPath(results2.second[results2.second.size() - 1]);
    }
    else
    {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output3.avi!          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '5':
    break;
  default:
    std::cout << "Please select 1 - 5." << std::endl;
    PrintMenu();
    break;
  }
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile()
{
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line))
  {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ','))
    {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else
      {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++)
  {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids)
  {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress)
{
  cv::VideoWriter video("src/lib/output10.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(1248, 992));
  for (auto location_ids : path_progress)
  {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++)
    {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                 cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
               cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
               LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
  video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon)
{
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/////////////////// Step 1 /////////////////////////////
/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name)
{
  const int size = name.size(); //length of string

  // Convert all of input to lower-case
  std::for_each(name.begin(), name.end(), [](char &c) { c = std::tolower(c); });

  std::vector<std::string> results;

  for (auto elem : data)
  {
    // convert data to lowercase for comparison purposes only
    std::string casename = elem.second.name;
    std::for_each(casename.begin(), casename.end(), [](char &c) { c = std::tolower(c); });
    // loop through the first n characters to compare. n is the input size.
    for (int i = 0; i < size; i++)
    {
      if (casename[i] != name[i])
        break;
      if (i == size - 1)
        results.push_back(elem.second.name); // return full name with proper case
    }
  }
  return results;
}

//////////////////// Step 2 ///////////////////////////

/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id)
{
  for (auto elem : data)
  {
    if (elem.second.id == id)
    {
      return elem.second.lat;
    }
  }
  return -1;
}

/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id)
{
  for (auto elem : data)
  {
    if (elem.second.id == id)
    {
      return elem.second.lon;
    }
  }
  return -1;
}

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id)
{
  for (auto elem : data)
  {
    if (elem.second.id == id)
    {
      return elem.second.name;
    }
  }
  return ""; //returns empty string if id does not exist
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id)
{
  std::vector<std::string> result;
  for (auto elem : data)
  {
    if (elem.second.id == id)
    {
      return elem.second.neighbors;
    }
  }
  return result; //returns empty vector if id does not exist
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name)
{

  std::pair<double, double> results(-1, -1);
  if (name == "")
    return results;
  for (auto elem : data)
  {
    if (elem.second.name == name)
    {
      std::pair<double, double> latlon(GetLat(elem.second.id), GetLon(elem.second.id));
      return latlon;
    }
  }
  return results;
}

////////////////////////////////////////////////////////////////////////////

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b)
{
  // TODO: Use Haversine (great Circle) Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles

  // radians = (degrees) * (pi / 180) to convert degrees to radians
  const double deg2rad = 3.14159265358979323846264338328 / 180.0;

  // Check if lat and lon inputs are valid
  if ((a.lon == -1 && a.lat == -1) || (b.lon == -1 && (b.lat == -1)))
    return 0;
  // Convert all positions to radians
  double alat = a.lat * deg2rad;
  double alon = a.lon * deg2rad;
  double blat = b.lat * deg2rad;
  double blon = b.lon * deg2rad;

  // Compute Distance using formula
  double dlat = blat - alat;
  double dlon = blon - alon;

  double A = (pow((sin(dlat / 2.0)), 2.0) + cos(alat) * cos(blat) * pow((sin(dlon / 2)), 2.0));
  double c = 2 * asin(std::min(1.0, sqrt(A)));
  return 3961 * c;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path)
{
  double sum = 0;
  const int i_last = path.size();
  // Loop - starts at 1 to find the distance between element 1 and element 0.
  for (int i = 1; i < i_last; i++)
  {
    // Create new node instance for two adjacent points
    Node a;
    Node b;

    // Find  and fill lat/lon data
    //Using GetLat GetLon separately, as path vector of IDs, which means GetPosition could not be efficiently used.
    for (auto elem : data)
    {
      if (elem.second.id == path[i - 1])
      {
        std::pair<double, double> apos(GetLat(elem.second.id), GetLon(elem.second.id));
        a.lat = apos.first;
        a.lon = apos.second;
      }
      if (elem.second.id == path[i])
      { // not using else if, as it is possible for two adjacent points to be identical
        std::pair<double, double> bpos(GetLat(elem.second.id), GetLon(elem.second.id));

        b.lat = bpos.first;
        b.lon = bpos.second;
      }
    }

    //Calculate distance
    sum += CalculateDistance(a, b);
  }

  return sum;
}

/**
 * CalculateShortestPath: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath(std::string location1_name, std::string location2_name)
{ //Dijkstra
  std::vector<std::string> path;
  // Get ID based from Location Names
  std::string location1_id, location2_id;
  for (auto elem : data)
  {
    if (elem.second.name == location1_name)
      location1_id = elem.second.id;
    else if (elem.second.name == location2_name)
      location2_id = elem.second.id;
  }
  Node point1 = data[location1_id];
  Node point2 = data[location2_id];
  //build adjacency list
  std::map<std::string, std::vector<std::pair<std::string, double>>> adjacency_list;
  for (auto item : data)
  {
    for (auto neighbor : item.second.neighbors)
    {
      adjacency_list[item.second.id].push_back(std::pair<std::string, double>(neighbor, CalculateDistance(item.second, data[neighbor])));
    }
  }
  typedef std::pair<double, std::pair<std::string, std::string>> pq_node;
  std::priority_queue<pq_node, std::vector<pq_node>, std::greater<pq_node>> pq; //{dist, (prev_node,current_node)}

  //add starting node's neighbors to priority queue
  for (auto x : adjacency_list[point1.id])
  {
    pq.push(make_pair(x.second, std::pair<std::string, std::string>(point1.id, x.first))); //{total_dist,{prev_node, next_node}}
  }
  // for(auto x : data[point1.id].neighbors)
  // {
  //   pq.push(make_pair(CalculateDistance(data[point1.id],data[x]), std::pair<std::string,std::string>(point1.id, x))); //{total_dist,{prev_node, next_node}}
  // }
  //add starting node to visited_nodes map
  std::map<std::string, double> visited_nodes = {{point1.id, 0.0}}; //total distance to the dest. node (key)
  std::map<std::string, std::string> direction_map;                 //input dest. node (key) and value is prev node on shortest path

  pq_node current;
  int count = 0;
  while (!pq.empty())
  {
    count++;
    //pop shortest path node off priority queue
    current = pq.top();
    pq.pop();

    if (!visited_nodes.count(current.second.second)) //unvisited node
    {
      visited_nodes[current.second.second] = current.first;        //add to visited nodes {current_node: total_dist}
      direction_map[current.second.second] = current.second.first; //add to direction map {current_node: prev_node}
    }
    else
    {
      if ((current.first) < visited_nodes[current.second.second])
      {
        visited_nodes[current.second.second] = current.first;
        direction_map[current.second.second] = current.second.first;
      }
    }
    for (auto neighbor : adjacency_list[current.second.second])
    {
      if (neighbor.first != current.second.first && !visited_nodes.count(neighbor.first)) //dont add neighbor that node came from or that has been visited
      {
        //Add node to priority queue
        //Node: {total_dist, {current_node, next_node}}
        pq.push(make_pair(neighbor.second + visited_nodes[current.second.second], std::pair<std::string, std::string>(current.second.second, neighbor.first)));
      }
    }
    // for(auto neighbor : data[current.second.second].neighbors)
    // {
    //   if(neighbor != current.second.first && !visited_nodes.count(neighbor))
    //   {
    //     pq.push(make_pair(CalculateDistance(data[neighbor],data[current.second.second]) + visited_nodes[current.second.second], std::pair<std::string,std::string>(current.second.second, neighbor)));
    //   }
    // }
  }

  //use direction map to create path from source to destination
  std::string temp;
  std::string prev_node = point2.id;
  while (prev_node != point1.id)
  {
    path.push_back(prev_node);
    temp = direction_map[prev_node];
    prev_node = temp;
  }
  path.push_back(prev_node);
  std::reverse(path.begin(), path.end()); //list returned is destination->source so we reverse it

  std::cout << "Iterations: " << count << std::endl;
  std::cout << "Direct path distance: " << CalculateDistance(point1, point2) << " miles" << std::endl;
  std::cout << "Path length: " << CalculatePathLength(path) << " miles" << std::endl;
  std::cout << "Nodes along path: " << path.size() << std::endl;
  // for(auto i:path) std::cout<<i<<"\t";
  // PlotPath(path);
  return path;
}

//Helper Debug
void TrojanMap::printPath(std::vector<int> parent, int j)
{
  if (parent[j] == -1)
    return;
  printPath(parent, parent[j]);
  std::cout << j << " ";
}

/**
 * Travelling salesman problem: Given a lis t of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(std::vector<std::string> &location_ids)
{
  // Using Brute Force
  std::vector<std::string> lid;
  lid = location_ids;
  // to store minimum distance
  std::pair<double, std::vector<std::vector<std::string>>> results;
  results.first = 0;
  int i = 0;
  std::cout << "\n ";
  do
  {
    // Calculate Path Length
    std::vector<std::string> lid_circular;
    lid_circular = lid;
    lid_circular.push_back(lid_circular.at(0));
    double d = CalculatePathLength(lid_circular);
    // If distance less than past permutation (or first permutation), update
    if (d < results.first || results.first == 0)
    {
      results.first = d;
      results.second.push_back(lid_circular);
    }
    PlotPath(lid_circular);
    int size = lid.size(), total_itera = 1;
    for (int i = size; i > 0; i--)
      total_itera *= i;
    std::cout << "\rIteration: " << ++i << "/" << total_itera << std::flush;
  } while (std::next_permutation(lid.begin(), lid.end()));

  return results;
}

// Using 2-OPT method

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
    std::vector<std::string> &location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> results;
  results.second.push_back(location_ids);

  int improvement = 0;
  int size = location_ids.size();
  double min_dist;

  while (improvement < 50) //check whether distance improvement is made
  {
    min_dist = CalculatePathLength(location_ids);
    for (int i = 0; i < size - 1; i++)
    {
      for (int k = i + 1; k < size; k++)
      {
        // New iteration
        std::vector<std::string> id_new;
        twoOptSwap(i, k, location_ids, id_new);
        double new_distance = CalculatePathLength(id_new);

        if (new_distance < min_dist)
        {
          improvement = 0;
          location_ids = id_new;
          min_dist = new_distance;
          results.second.push_back(location_ids);
        }
      }
    }
    PlotPath(location_ids);
    improvement++;
  }
  results.first = min_dist;
  for (int i = 0; i < results.second.size(); i++)
  {
    results.second[i].push_back(location_ids[0]);
  }

  return results;
}

void TrojanMap::twoOptSwap(const int &i, const int &k, std::vector<std::string> &route, std::vector<std::string> &new_path)
{

  int size = route.size();
  for (int curr = 0; curr < i; curr++)
  {
    new_path.push_back(route[curr]);
  }

  for (int curr = k; curr >= i; curr--)
  {
    new_path.push_back(route[curr]);
  }

  for (int curr = k + 1; curr < size;curr --)
  {
    new_path.push_back(route[curr]);
  }
}
/**
 * Debug function to show the values of data.  Not used in final program.
 * 
 * @param None
 * @return {std::map<std::string, Node> data_out : a map of all data from map.csv}
 */
std::map<std::string, Node> TrojanMap::data_out()
{
  return data;
}