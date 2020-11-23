// UNUSED FUNCTION. 

 std::vector<std::string> x;
  std::string location1_id, location2_id;
  const int INF = INT_MAX; //infinity

  // Check empty inputs
  if (location1_name == "" || location2_name == "")
    return x;
  // Get ID based from Location Names
  for (auto elem : data)
  {
    if (elem.second.name == location1_name)
      location1_id = elem.second.id;
    else if (elem.second.name == location2_name)
      location2_id = elem.second.id;
  }

  // Adjacency/Neighbor List of Neighbors
  std::vector<std::string> na, nb;
  na = GetNeighborIDs(location1_id);
  nb = GetNeighborIDs(location2_id);

  // Debug Neighbor ID
  // for (auto const i : na)
  //   std::cout << i << " ";
  // for (auto const i : nb)
  //   std::cout << i << " ";

  // Adjacency Matrix Construction
  std::vector<std::vector<double>> adjacency;
  int location1_code, location2_code;
  std::map<std::string, Node>::iterator it = data.begin();
  int i_ = 0;
  // key order pair
  std::vector<std::pair<std::string, int>> lid; //location id
  while (it != data.end())
  {
    lid.push_back(std::pair<std::string, int>(it->first, i_));
    if (location1_id == it->first)
      location1_code = i_;
    if (location2_id == it->first)
      location2_code = i_;
    ++i_;
    ++it;
  }

  it = data.begin();
  int id1 = 0;
  while (it != data.end())
  { //for each location A
    adjacency.push_back(std::vector<double>(i_ + 1, 0));
    // std::cout<<":"<<it->first << ":";
    for (auto i_ : (it->second).neighbors)
    { //for each neighbors of location A
      int id2 = 0;
      for (auto i_lid : lid)
      { //search location id
        if (i_lid.first == i_)
        {
          id2 = i_lid.second;
          // std::cout<<"-->" <<i_ << "\n";
          break;
        } //set location id
      }
      // Determine Distance
      std::vector<std::string> path{it->first, i_};
      double d = CalculatePathLength(path);
      // Insert distance
      adjacency[id1].at(id2) = d; //.insert(adjacency.begin()+id2, d);
    }
    ++id1;
    ++it;
  }
  // Initialize infinity for disconnected paths
  // for (int i = 0; i < adjacency.size(); i++)
  // {
  //   for (int j = 0; j < adjacency[i].size(); j++)
  //   {
  //     if (i != j && adjacency[i].at(j) == 0)
  //       adjacency[i].at(j) = INF;
  //   }
  // }

  // Initialize distances, parent nodes, visited flag
  std::vector<double> distance(i_ + 1, INF);
  std::vector<int> parent(i_ + 1, -1);
  std::vector<bool> visited(i_ + 1, false);

  // Distance from source to itself is 0
  distance.at(location1_code) = 0;

  // Find shortest path for all available vertices
  for (int count = 0; count < i_ + 1; count++)
  {
    int min_index, min = INF;

    for (int v = 0; v < i_; v++)
    {
      if (visited[v] == false && distance[v] <= INF){
        min_index = v;
        min = distance[v];
        }
    }
    // Mark as visited
    visited[min_index] = true;

    // Update distances of the selected vertex
    for (int v = 0; v < i_; v++)
    {
      int a = (distance[min_index] + adjacency[min_index].at(v) < distance[v])?1:0;
      if(a==1) std::cout<<a<<std::endl;
      // Update only 1) not visited 2) edge exists 3) new distance smaller than current distance
      if (!visited[v] && adjacency[min_index].at(v) && distance[min_index] + adjacency[min_index].at(v) < distance[v])
      {
        parent[v] = min_index;
        distance.at(v) = distance[min_index] + adjacency[min_index].at(v);
        // std::cout<<distance[min_index]<<std::endl;
      }
    }
   // if(distance[location2_code] < INF) std::cout<<distance[location2_code] << std::endl;
  }

  // std::cout << location1_code << "->"<<location2_code <<"distance"<< distance[location2_code] << "path";
  // printPath(parent, location1_code);
  // for(auto i: distance) {if(i!=INF)std::cout<<i<<",";}
  // int limit = 0;
  // int z = 0;
  // for(auto x : adjacency){
  //   std::cout<< z++ << "\t";
  //   for(auto y : x){
  //     std::cout << y << "\t";
  //   }
  //   std::cout <<"\n";
  //   if (z > limit) break;
  // }
  // //       return x;

  return x;