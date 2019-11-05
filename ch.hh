#ifndef __HEAP__HH
#define __HEAP__HH

#include <algorithm>
#include <functional>
#include <vector>
#include <queue>
#include <sstream>

struct Shortcut {
  int from;
  int to;
  int cost;

  bool operator==(const Shortcut& t) const {
    return from == t.from && to == t.to && cost == t.cost;
  }

  std::string getKey() {
    std::stringstream ss;
    ss << from << "," << to << "," << cost;
    return ss.str();
  }
};

struct ShortCutHash {
  size_t operator()(const Shortcut& t) const {
    return std::hash<int>()(t.from) ^ std::hash<int>()(t.to) ^ std::hash<int>()(t.cost);
  } 
};

struct NodeImportance {
  int nodeId;
  int importance;
  bool operator== (const NodeImportance& p) const {
    return p.nodeId == nodeId;
  }
};

struct compareNodeImp {
  bool operator()(NodeImportance &a, NodeImportance &b) { return a.importance > b.importance; }
};

struct TargetInfo {
  int nodeId;
  int cost;
  bool operator== (const TargetInfo& info) const {
    return cost == info.cost;
  }
};

struct compareTo {
  bool operator()(TargetInfo &a, TargetInfo &b) { return a.cost > b.cost; }
};

template<typename T, class Container=std::vector<T>, class Compare=std::less<typename Container::value_type>> 
class heap : public std::priority_queue<T, Container, Compare> {
public:
  bool erase(const T& value) {
    auto it = std::binary_search(this->c.begin(), this->c.end(), value);
    if (it != this->c.end()) {
      this->c.erase(it);
      std::make_heap(this->c.begin(), this->c.end(), this->comp);
      return true;
    } else {
      return false;
    }
  }

  bool erase(const T&& value) {
    auto it = std::find(this->c.begin(), this->c.end(), value);
    if (it != this->c.end()) {
      this->c.erase(it);
      std::make_heap(this->c.begin(), this->c.end(), this->comp);
      return true;
    } else {
      return false;
    }
  }
};

template<typename T>
std::vector<T> combineVec(std::vector<T>& a, std::vector<T>& b) {
  std::vector<T> vec;
  vec.reserve(a.size() + b.size());
  vec.insert(vec.end(), a.begin(), a.end());
  vec.insert(vec.end(), b.begin(), b.end());
  return vec;
}

#endif
