#include "ch.hh"
#include <cassert>
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <chrono>
#include <unistd.h>

#define FILENAME "/code/graph"
#define MAX_HOPS 3
// #define __GR_FILE__ "/code/USA-road-t.NY.gr"
#define __UNDIRECTED_GRAPH__
#define __DEBUG__

using namespace std;

class Graph {
  int nodesCount;
  vector<vector<TargetInfo>> outgoing_edges;
  vector<vector<TargetInfo>> incoming_edges;
  // Levels of nodes for node ordering, <nodeId, importance>
  unordered_map<int, int> nodeOrders;
  // Levels of nodes for node ordering, <nodeId, importance>
  unordered_map<int, unordered_set<Shortcut, ShortCutHash>> augmentedGraph;
  unordered_map<string, Shortcut> shortcutMap;
public:
  Graph() {
    // read_stdin();
    read_file();
#ifdef __DEBUG__
    auto start = std::chrono::high_resolution_clock::now();
    cout << "Preprocessing ..." << endl;
#endif
    preprocess();
#ifdef __DEBUG__
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    cout << "Preprocessing time: "<< std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms" << endl;
    start = std::chrono::high_resolution_clock::now();
#endif
    finalize();
#ifdef __DEBUG__
    elapsed = std::chrono::high_resolution_clock::now() - start;
    cout << "Finalize time: "<< std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms" << endl;
#endif
  }

  vector<TargetInfo> &get_forward_neighbors(int v) {
    return incoming_edges[v];
  }

  vector<TargetInfo> &get_backward_neighbors(int v) {
    return outgoing_edges[v];
  }

  vector<TargetInfo> get_forward_uncontracted_neighbors(int v) {
    return get_uncontracted_neighbors(incoming_edges, v);
  }

  vector<TargetInfo> get_backward_uncontracted_neighbors(int v) {
    return get_uncontracted_neighbors(outgoing_edges, v);
  }

  void preprocess();
  // Returns distance from s to t in the graph
  int query(int from, int to);
private:
  // Try to relax the node v using distance d either in the forward or in the
  // backward search
  void update(int& estimateCost,
              int& meetingPoint,
              heap<TargetInfo, vector<TargetInfo>, compareTo>& costQueue,
              unordered_set<int>& visited,
              unordered_set<int>& visitedB,
              unordered_map<int, int>& distRecord,
              unordered_map<int, int>& parents);
  
  void finalize();

  pair<int, vector<Shortcut>> calcImportance(int nodeId);

  unordered_set<int> visited;
  // Adds all the shortcuts for the case when node v is contracted, and returns
  // the importance of node v in this case (number of shortcuts)
  vector<Shortcut> get_shortcuts(int v);
  unordered_set<int> dijkstra(int compressingNode, int from, unordered_set<int> to, int capCost);

  void setNodesCount(int n) {
    nodesCount = n;
    outgoing_edges.resize(n);
    incoming_edges.resize(n);
  }

  vector<TargetInfo> get_uncontracted_neighbors(vector<vector<TargetInfo>>& edges, int v) {
    vector<TargetInfo> vec;
    for (auto p : edges[v]) {
      if (visited.find(p.nodeId) == visited.end()) {
        vec.push_back(p);
      }
    }
    return vec;
  }

  void addEdgeToList(vector<TargetInfo> &list, int target, int cost) {
    for (int i = 0; i < list.size(); ++i) {
      TargetInfo &path = list[i];
      if (path.nodeId == target) {
        if (path.cost > cost) {
          path.cost = cost;
        }
        return;
      }
    }
    list.push_back({ target, cost });
  }

  void add_directed_edge(int from, int to, int cost) {    
    addEdgeToList(outgoing_edges[from], to, cost);
    addEdgeToList(incoming_edges[to], from, cost);
  }

  void add_edge(int from, int to, int cost) { 
    add_directed_edge(from, to, cost); 
    add_directed_edge(to, from, cost);
  }

  void read_file() {
#ifdef __GR_FILE__
    ifstream fs(string(getcwd(nullptr, 0)) + __GR_FILE__);
#else
    int pathCount = 0;
    ifstream fs(string(getcwd(nullptr, 0)) + FILENAME);
#endif
    string line;
    int from, to, cost;
#ifdef __GR_FILE__
    while (getline(fs, line)) {
      istringstream iss(line);
      string lineType;
      iss >> lineType;
      assert(lineType == "c" || lineType == "p" || lineType == "a");
      if (lineType == "c") continue;
      if (lineType == "p") {
        string tmp;
        assert((iss >> tmp >> nodesCount) && tmp == "sp");
        incoming_edges = vector<vector<TargetInfo>>(nodesCount);
        outgoing_edges = vector<vector<TargetInfo>>(nodesCount);
      }
      if (lineType == "a") {
        assert((iss >> from >> to >> cost));
        add_directed_edge(from - 1, to - 1, cost);
      }
    }
#else
    getline(fs, line); // verticesNum, lineNum
    istringstream iss(line);
    assert((iss >> nodesCount >> pathCount));
    incoming_edges = vector<vector<TargetInfo>>(nodesCount);
    outgoing_edges = vector<vector<TargetInfo>>(nodesCount);
  
    for (int i = 0; i < pathCount; i++) {
      getline(fs, line);
      istringstream iss(line);
      assert((iss >> from >> to >> cost));
      add_edge(from, to, cost);
    }
#endif
  }

  bool read_stdin() {
    int from, to, cost, nodesCount, pathsCount;
    assert(scanf("%d %d", &nodesCount, &pathsCount) == 2);
    setNodesCount(nodesCount);
    for (int i = 0; i < pathsCount; ++i) {
      assert(scanf("%d %d %d", &from, &to, &cost) == 3);
      add_edge(from - 1, to - 1, cost);
    }
    finalize();
    return true;
  }
};

/**
 * 1. edge difference: num_of_shortcuts - num_of_edges
 * 2. deleted neighbors: num_of_contracted_neighbors (including neighbors via shortcuts)
 * 3. Voronoi Regions: every nodes u that are closer to v than w
 * 4. cost of contraction: num_of_shortcuts required
 * 5. cost of query: when node v is contracted, we increase all of v' neighbors
 */
pair<int, vector<Shortcut>> Graph::calcImportance(int nodeId) {
  const auto shortcuts = get_shortcuts(nodeId);
  int deletedNeighbors = 0;
#ifdef __UNDIRECTED_GRAPH__
  const int edgeDiff = shortcuts.size() - get_forward_neighbors(nodeId).size();
#else
  const int importance = shortcuts.size() - get_forward_neighbors(nodeId).size() - get_backward_neighbors(nodeId).size();
#endif
  for (auto node : outgoing_edges[nodeId]) {
    if (visited.find(node.nodeId) != visited.end()) {
      deletedNeighbors++;
    }
  }
  return { edgeDiff + deletedNeighbors, shortcuts };
}

/**
 * process phase, which we calculate the importance of nodes and contraction them
 * Lazy update: 
 * 1. before contracting v, re-evaluate its priority function, if newCost > secondSmallestCost,
 * reinsert v to the queue
 * 2. recompute the priority for all the neighbors when contraction
 * 3. it might be a good idea to recompute all priorities from time to time
 */
void Graph::preprocess() {
  // Priority queue will store pairs of (node, importance) with the least important node in the head
  heap<NodeImportance, vector<NodeImportance>, compareNodeImp> queue;
  for (int nodeId = 0; nodeId < nodesCount; nodeId++) {
    const int importance = calcImportance(nodeId).first;
    nodeOrders[nodeId] = importance;
    queue.push({ nodeId, importance });
  }
  while (!queue.empty()) {
    // TO DO: periodically update all uncontracted nodes
    auto curr = queue.top();
    queue.pop();
    const int nodeId = curr.nodeId;
#ifdef __DEBUG__
    if (nodesCount < 1000) {
      cout << "Contracting node " << nodeId << " ... " << endl;
    } else if (visited.size() % 500 == 0) {
      cout << ((visited.size() / (double)nodesCount) * 100.0) << "%" << endl;
    }
#endif
    const auto calcResult = calcImportance(nodeId);
    const int recalcCurrImportance = calcResult.first;
    nodeOrders[nodeId] = recalcCurrImportance;
    const auto shortcuts = calcResult.second;
    const int secondLeastImportance = queue.top().importance;
    if (recalcCurrImportance > secondLeastImportance) {
      queue.push({ nodeId, recalcCurrImportance });
    } else {
      // contract here
      for (auto shortcut : shortcuts) {
        shortcutMap[shortcut.getKey()] = shortcut;
#ifdef __UNDIRECTED_GRAPH__
        add_edge(shortcut.from, shortcut.to, shortcut.cost);
#else
        add_directed_edge(shortcut.from, shortcut.to, shortcut.cost);
#endif
      }
      visited.insert(nodeId);
      // reorder here
      auto forwardNodes = get_forward_uncontracted_neighbors(nodeId);
#ifdef __UNDIRECTED_GRAPH__
      for (auto neighbor : forwardNodes) {
#else
      auto backwardNodes = get_backward_uncontracted_neighbors(nodeId);
      for (auto neighbor : combineVec(forwardNodes, backwardNodes)) {
#endif
        const int importance = calcImportance(neighbor.nodeId).first;
        if (nodeOrders[neighbor.nodeId] != importance) {
          nodeOrders[neighbor.nodeId] = importance;
          queue.erase({ neighbor.nodeId, 0 });
          queue.push({ neighbor.nodeId, importance });
        }
      }
    }
  }
}

vector<Shortcut> Graph::get_shortcuts(int v) {
  int maxCost = 0;
  unordered_set<int> destNodes;
  vector<Shortcut> shortcuts;
  for (auto edge : incoming_edges[v]) {
    if (visited.find(edge.nodeId) == visited.end()) {
      destNodes.insert(edge.nodeId);
      maxCost = max(maxCost, edge.cost);
    }
  }

  for (auto edge : outgoing_edges[v]) {
    if (visited.find(edge.nodeId) == visited.end()) {
      auto hasWitnessNodes = dijkstra(v, edge.nodeId, destNodes, edge.cost + maxCost);
      for (auto targetNode : destNodes) {
        if (targetNode == edge.nodeId || hasWitnessNodes.find(targetNode) != hasWitnessNodes.end()) {
          continue;
        }
        auto from = find_if(incoming_edges[v].begin(), incoming_edges[v].end(), [&edge](const TargetInfo& obj) {
          return obj.nodeId == edge.nodeId;
        });
        auto to = find_if(outgoing_edges[v].begin(), outgoing_edges[v].end(), [&targetNode](const TargetInfo& obj) {
          return obj.nodeId == targetNode;
        });
        auto fromCost = incoming_edges[v][distance(incoming_edges[v].begin(), from)].cost;
        auto toCost = outgoing_edges[v][distance(outgoing_edges[v].begin(), to)].cost;
        shortcuts.push_back({
          .from = edge.nodeId, 
          .to = targetNode, 
          .cost = fromCost + toCost
        });
      }
    }
  }
#ifdef __VERBOSE__
  cout << "Contracting node " << v << ", found " << shortcuts.size() << " shortcuts" << endl;
#endif
  return shortcuts;
}

/**
 * one-to-many dijkstra search
 * @returns <nodeId, hasWitness>
 */
unordered_set<int> Graph::dijkstra(int compressingNode, int from, unordered_set<int> to, int capCost) {
  // <nodeId, <hops, cost>>
  unordered_map<int, pair<int, int>> destCost;
  unordered_map<int, int> parents;
  priority_queue<TargetInfo, vector<TargetInfo>, compareTo> minCostQueue;
  unordered_set<int> hasWitnessNodes;
  destCost[from] = { 0, 0 };
  minCostQueue.push({ .nodeId = from, .cost = 0 });
  if (to.find(from) != to.end()) {
    to.erase(from);
  }

  while (!minCostQueue.empty()) {
    TargetInfo curr = minCostQueue.top();
    minCostQueue.pop();
    const auto currInfo = destCost[curr.nodeId];
    const int currHops = currInfo.first;
    const int currCost = currInfo.second;

    if (curr.cost > capCost) {
      continue;
    }
    if (to.find(curr.nodeId) != to.end()) {
      to.erase(curr.nodeId);
      hasWitnessNodes.insert(curr.nodeId);
      if (to.empty()) {
        return hasWitnessNodes;
      }
    }
    if (currHops == MAX_HOPS) {
      continue;
    }

    for (auto targetNode : incoming_edges[curr.nodeId]) {
      // if targetNode.contractOrder < curr.contractOrder; then break;
      const int next = targetNode.nodeId;
      // always ignore node to be contract
      if (next == compressingNode) {
        continue;
      }
      const int nextCost = targetNode.cost;
      const int nextDestCost = currCost + nextCost;
      if (destCost.find(next) == destCost.end() || nextDestCost < destCost[next].second) {
        auto& destInfo = destCost[next];
        destInfo.first = currHops + 1;
        destInfo.second = nextDestCost;
        minCostQueue.push({ next, nextDestCost });
        parents[next] = curr.nodeId;
      }
    }
  }
  return hasWitnessNodes;
}

void Graph::finalize() {
  for (auto i = 0; i < incoming_edges.size(); i++) {
    for (auto to : incoming_edges[i]) {
      augmentedGraph[i].insert({ i, to.nodeId, to.cost });
    }
  }
  for (auto i = 0; i < outgoing_edges.size(); i++) {
    for (auto to : outgoing_edges[i]) {
      augmentedGraph[i].insert({ i, to.nodeId, to.cost });
    }
  }
  incoming_edges.clear();
  outgoing_edges.clear();
}

int Graph::query(int u, int w) {
  unordered_map<int, int> forwardDist;
  unordered_map<int, int> backwardDist;
  unordered_set<int> forwardVisited;
  unordered_set<int> backwardVisited;
  heap<TargetInfo, vector<TargetInfo>, compareTo> forwardCostQueue;
  heap<TargetInfo, vector<TargetInfo>, compareTo> backwardCostQueue;
  unordered_map<int, int> forwardParent;
  unordered_map<int, int> backwardParent;
  forwardDist.insert({ u, 0 });
  backwardDist.insert({ w, 0 });
  forwardVisited.insert(u);
  backwardVisited.insert(w);
  forwardCostQueue.push({ u, 0 });
  backwardCostQueue.push({ w, 0 });
  int meetingPoint = -1;
  int estimateCost = INT_MAX;
  while (!(forwardCostQueue.empty() && backwardCostQueue.empty())) {
    int currCost = 0;
    if (!forwardCostQueue.empty()) {
      currCost += forwardCostQueue.top().cost;
    }
    if (!backwardCostQueue.empty()) {
      currCost += backwardCostQueue.top().cost;
    }
    if (currCost >= estimateCost) {
      return estimateCost;
    }
    update(estimateCost, meetingPoint, forwardCostQueue, forwardVisited, backwardVisited, forwardDist, forwardParent);
    update(estimateCost, meetingPoint, backwardCostQueue, backwardVisited, forwardVisited, backwardDist, backwardParent);
  }
  return estimateCost;
}

void Graph::update(
    int& estimateCost,
    int& meetingPoint,
    heap<TargetInfo, vector<TargetInfo>, compareTo>& costQueue,
    unordered_set<int>& visited,
    unordered_set<int>& visitedB,
    unordered_map<int, int>& distRecord,
    unordered_map<int, int>& parents) {
  if (costQueue.empty()) {
    return;
  }
  const auto currPath = costQueue.top();
  const int currOrder = nodeOrders[currPath.nodeId];
  costQueue.pop();
  visited.insert(currPath.nodeId);
  for (auto path : augmentedGraph[currPath.nodeId]) {
    const int nextOrder = nodeOrders[path.to];
    if (nextOrder < currOrder) {
      continue;
    }
    const int nextCost = path.cost + distRecord[currPath.nodeId];
    if (distRecord.find(path.to) == distRecord.end() || nextCost < distRecord[path.to]) {
      distRecord[path.to] = nextCost;
      parents[path.to] = currPath.nodeId;
      costQueue.push({ path.to, nextCost });
      if (visitedB.find(path.to) != visitedB.end() && estimateCost > nextCost) {
        estimateCost = nextCost;
        meetingPoint = path.to;
      }
    }
  }
}

int main() {
  Graph graph;
#ifdef __DEBUG__
  auto start = std::chrono::high_resolution_clock::now();
#endif
  cout << graph.query(0, 4) << endl;
#ifdef __DEBUG__
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  cout << "Query time: " << std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() << " ms" << endl;
#endif
  // cout << "Ready" << endl;
  // int testSize;
  // assert(scanf("%d", &testSize) == 1);
  // for (int i = 0; i < testSize; ++i) {
  //   int from, to;
  //   assert(scanf("%d %d", &from, &to) == 2);
  //   printf("%d\n", graph.query(from - 1, to - 1));
  // }
}
