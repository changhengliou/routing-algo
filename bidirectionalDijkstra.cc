#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <cassert>
#include <chrono>
#include <thread>
#include <mutex>
#include <random>

using namespace std;

struct path {
  int to;
  int cost;
};

struct compareTo {
  bool operator()(path &a, path &b) { return a.cost > b.cost; }
};

constexpr auto default_potential_func = [](int a, int b) { return 0; };
constexpr auto print_func = [](vector<int> vec) {
  for (auto i = 0; i < vec.size(); i++) {
    std::cout << vec[i] << " ";
    if (vec.size() > 10 && i == 4) {
      i = vec.size() - 5;
      std::cout << "... ";
    }
  }
  std::cout << std::endl;
};
int manhattanDist(int from, int to);

vector<int> landmarks;
vector<vector<int>> scores;
vector<vector<path>> graph;
vector<pair<int, int>> coordinates;
const string grFile = "/Users/line/Desktop/changheng/prep/code/USA-road-d.West.gr"; // USA-road-d.E.co // USA-road-d.NY
const string coFile = "/Users/line/Desktop/changheng/prep/code/USA-road-d.West.co"; // USA-road-d.West
int maxWeight = 0;

void writePath(int from, int to, vector<int>& v) {
  ofstream f;
  f.open("/Users/line/Desktop/changheng/prep/code/USA-road-d.NY.path", ios::app);
  f << from << ", " << to << "\n";
  for (auto i : v) {
    f << i << ", ";
  }
  f << "\n";
  f.close();
}

pair<int, vector<int>> dijkstra(int from, int to, int (*potentialFunc)(int, int)) {
  vector<int> parent(graph.size(), -1);
  vector<int> costRecord(graph.size(), INT_MAX);
  priority_queue<path, vector<path>, compareTo> minCostQueue;
  costRecord[from] = 0;
  minCostQueue.push({from, 0});
  while (!minCostQueue.empty()) {
    path currPath = minCostQueue.top();
    minCostQueue.pop();
    if (currPath.to == to) {
      return { costRecord[to], {} };
      vector<int> outputPath;
      int curr = to;
      while (curr != from) {
        outputPath.push_back(curr);
        curr = parent[curr];
        assert(curr != -1);
      }
      outputPath.push_back(from);
      reverse(outputPath.begin(), outputPath.end());
      return { costRecord[to], outputPath };
    }
    for (auto nextPath : graph[currPath.to]) {
      const int dest = nextPath.to;
      const int cost = nextPath.cost;
      const int newDestCost = costRecord[currPath.to] + cost + potentialFunc(dest, to);
      if (newDestCost < costRecord[dest]) {
        costRecord[dest] = newDestCost;
        minCostQueue.push({ dest, newDestCost });
        parent[dest] = currPath.to;
      }
    }
  }
  return {};
}

vector<int> dijkstra(int from) {
  vector<int> costRecord(graph.size(), INT_MAX);
  priority_queue<path, vector<path>, compareTo> minCostQueue;
  costRecord[from] = 0;
  minCostQueue.push({from, 0});
  while (!minCostQueue.empty()) {
    path currPath = minCostQueue.top();
    minCostQueue.pop();
    for (auto nextPath : graph[currPath.to]) {
      const int dest = nextPath.to;
      const int cost = nextPath.cost;
      const int newDestCost = costRecord[currPath.to] + cost;
      if (newDestCost < costRecord[dest]) {
        costRecord[dest] = newDestCost;
        minCostQueue.push({ dest, newDestCost });
      }
    }
  }
  return costRecord;
}

void relaxEdges(priority_queue<path, vector<path>, compareTo> &minCostQueue,
                vector<int> &parent,
                vector<int> &costRecord,
                vector<int> &costRecordB,
                vector<int> &visited, 
                vector<int> &visitedB,
                int &meetingPoint, 
                int &minCost,
                int &to,
                int (*potentialFunc)(int, int)) {
  const path currPath = minCostQueue.top();
  minCostQueue.pop();
  visited[currPath.to] = 1;
  for (auto nextPath : graph[currPath.to]) {
    const int dest = nextPath.to;
    const int cost = nextPath.cost;
    const int newDestCost = costRecord[currPath.to] + cost + potentialFunc(dest, to);
    if (newDestCost < costRecord[dest]) {
      parent[dest] = currPath.to;
      costRecord[dest] = newDestCost;
      minCostQueue.push({dest, newDestCost});
      if (visitedB[dest] != 0 && minCost > newDestCost) {
        minCost = newDestCost;
        meetingPoint = dest;
      }
    }
  }
}

pair<int, vector<int>> bidirectionalDijkstra(int from, int to, int (*potentialFunc)(int, int)) {
  vector<int> parentA(graph.size(), -1);
  vector<int> costRecordA(graph.size(), INT_MAX);
  vector<int> visitedA(graph.size(), 0);
  priority_queue<path, vector<path>, compareTo> minCostQueueA;
  vector<int> parentB(graph.size(), -1);
  vector<int> costRecordB(graph.size(), INT_MAX);
  vector<int> visitedB(graph.size(), 0);
  priority_queue<path, vector<path>, compareTo> minCostQueueB;
  costRecordA[from] = 0;
  minCostQueueA.push({from, 0});
  costRecordB[to] = 0;
  minCostQueueB.push({to, 0});
  int minCost = INT_MAX;
  int meetingPoint = -1;
  visitedA[from] = 1;
  visitedB[to] = 1;

  while (!minCostQueueA.empty() && !minCostQueueB.empty()) {
    const int currCost = minCostQueueA.top().cost + minCostQueueB.top().cost;
    if (currCost >= minCost) {
      vector<int> outputPath;
      // construct routing path of forward search
      int curr = meetingPoint;
      while (curr != from) {
        outputPath.push_back(curr);
        curr = parentA[curr];
        assert(curr != -1);
      }
      outputPath.push_back(from);
      reverse(outputPath.begin(), outputPath.end());
      // construct routing path of backward search
      if (meetingPoint != to) {
        curr = parentB[meetingPoint];
        while (curr != to) {
          outputPath.push_back(curr);
          curr = parentB[curr];
          assert(curr != -1);
        }
        outputPath.push_back(to);
      }
      return { costRecordA[meetingPoint] + costRecordB[meetingPoint], outputPath };
    }
    relaxEdges(minCostQueueA, parentA, costRecordA, costRecordB,
                visitedA, visitedB, meetingPoint, minCost, to, potentialFunc);
    relaxEdges(minCostQueueB, parentB, costRecordB, costRecordA,
                visitedB, visitedA, meetingPoint, minCost, to, potentialFunc);
  }
  return {};
}

void readCoordinateFile(vector<pair<int, int>> &coordinates) {
  coordinates = vector<pair<int, int>>(graph.size());
  ifstream fs(coFile); // USA-road-t.NY.co
  string line;
  int id, x, y;
  std::cout << "Reading coordinate file ..." << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  while (getline(fs, line)) {
    istringstream iss(line);
    string lineType;
    iss >> lineType;
    assert(lineType == "c" || lineType == "p" || lineType == "v");
    if (lineType == "v") {
      assert((iss >> id >> y >> x));
      coordinates[id - 1] = { x, y };
    }
  }
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  std::cout << __FUNCTION__ << ", Time Elapsed: " << (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000.0) << " ms" << endl;
}

void readAltCache() {
  ifstream f("/Users/line/Desktop/changheng/prep/code/USA-road-t.NY.score");
  auto start = std::chrono::high_resolution_clock::now();
  if (!f.is_open()) {
    std::cout << "Unable to read cached file..." << endl;
    return;
  }
  string line;
  while (getline(f, line)) {
    istringstream iss(line);
    int vertex, landmark, score;
    assert((iss >> vertex >> landmark >> score));
    scores[vertex][landmark] = score;
  }
  f.close();
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  std::cout << "Read cache completed ..., time elapsed = " << (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000.0) << " ms" << endl;
}

int getAltScore(int a, int b) {
  int maxCost = INT_MIN;
  for (auto i = 0; i < landmarks.size(); i++) {
    maxCost = max(max(scores[i][a] - scores[i][b], scores[i][b] - scores[i][a]), maxCost);
  }
  return maxCost;
}

pair<int, vector<int>> alt(int from, int to, int (*potentialFunc)(int, int)) {
  // readAltCache();
  return dijkstra(from, to, getAltScore);
}

double runWithStats(string funcName, pair<int, vector<int>> (*run)(int from, int to, int (*potentialFunc)(int, int)),
                    int from, int to, int (*potentialFunc)(int, int) = default_potential_func) {
  auto start = std::chrono::high_resolution_clock::now();
  auto result = run(from, to, potentialFunc);
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  const auto ms = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000.0;
  print_func(result.second);
  std::cout << funcName << ": Cost = " << result.first << ", Time Elapsed: " << ms  << " ms" << endl;
  return ms;
}

int loadFromFile() {
  ifstream fs(grFile);
  string line;
  int from, to, cost, nodesCount;
  int maxCost = INT_MIN;
  std::cout << "Reading files ..." << endl;
  auto start = std::chrono::high_resolution_clock::now();
  while (getline(fs, line)) {
    istringstream iss(line);
    string lineType;
    iss >> lineType;
    assert(lineType == "c" || lineType == "p" || lineType == "a");
    if (lineType == "c") continue;
    if (lineType == "p") {
      string tmp;
      assert((iss >> tmp >> nodesCount) && tmp == "sp");
      graph = vector<vector<path>>(nodesCount);
    }
    if (lineType == "a") {
      assert((iss >> from >> to >> cost));
      graph[from - 1].push_back({ to - 1, cost });
      maxCost = max(maxCost, cost);
    }
  }
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  std::cout << __FUNCTION__ << ", Time Elapsed: " << (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000.0) << " ms" << endl;
  return maxCost;
}

int manhattanDist(int from, int to) {
  const auto a = coordinates[from];
  const auto b = coordinates[to];
  // return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2)) / 50;
  return (abs(a.first - b.first) + abs(a.second - b.second)) / 50;
}

int main() {
  maxWeight = loadFromFile();
  readCoordinateFile(coordinates);

  std::random_device r;
  std::default_random_engine generator{r()};
  std::uniform_int_distribution<int> distribution(0, graph.size());
  vector<int> vec;
  // for (int i = 0; i < 8; i++) {
  //   vec.push_back(distribution(generator));
  // }
  landmarks = {5635846}; // NY 57574 E 2435138, 2829053, 383188 W 5635846
  for (int i = 0; i < landmarks.size(); i++) {
    cout << "Processing landmark " << landmarks[i] << endl;;
    const auto sc = dijkstra(landmarks[i]);
    scores.push_back(sc);
  }
  // alt(2294059, 2779821, default_potential_func);
  
  double totalTimeA = 0, totalTimeB = 0, totalTimeC = 0, totalTimeD = 0, totalTimeE = 0;
  for (auto i = 0; i < 20; i++) {
    const int from = distribution(generator);
    const int to = distribution(generator);
    totalTimeA += runWithStats("Dijkstra", &dijkstra, from, to);
    totalTimeB += runWithStats("Bidirectional Dijkstra", &bidirectionalDijkstra, from, to);
    totalTimeC += runWithStats("A star", &dijkstra, from, to, &manhattanDist);
    totalTimeD += runWithStats("Bidirectional A star", &bidirectionalDijkstra, from, to, &manhattanDist);
    totalTimeE += runWithStats("ALT", &alt, from, to);
  }
  cout << "Dijkstra :" << totalTimeA / 20.0 << endl;
  cout << "Bidirectional Dijkstra :" << totalTimeB / 20.0 << endl;
  cout << "A star :" << totalTimeC / 20.0 << endl;
  cout << "Bidirectional A star :" << totalTimeD / 20.0 << endl;
  cout << "ALT :" << totalTimeE / 20.0 << endl;
  return 0;
}
