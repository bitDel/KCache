#pragma once

#include <cstring>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "cache_policy.hpp"

namespace kcache {

template <typename Key, typename Value> class LruCache;

template <typename Key, typename Value>
class LruNode {

private:
  Key key_;
  Value value_;
  size_t accessCount_; // 访问次数
  std::weak_ptr<LruNode<Key, Value>> prev_; // 打破循环引用
  std::shared_ptr<LruNode<Key, Value>> next_;

public:
  LruNode(Key key, Value value)
    : key_(key),
      value_(value),
      accessCount_(1)
  {}

  Key getKey() const { return key_; }
  Value getValue() const { return value_; }
  void setValue(const Value &value) { value_ = value; }
  size_t getAccessCount() const { return accessCount_; }
  void incrementAccessCount() { ++ accessCount_; }

  friend class LruCache<Key, Value>;
};

template<typename Key, typename Value>
class LruCache : public cache_policy<Key, Value> {

public:
  using LruNodeType = LruNode<Key, Value>;
  using NodePtr = std::shared_ptr<LruNodeType>;
  using NodeMap = std::unordered_map<Key, NodePtr>;

  LruCache(int capacity)
    : capacity_(capacity) 
  {
    initializeList();
  }

  ~LruCache() override = default;

  void put(Key key, Value value) override {
    if (capacity_ <= 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodeMap_.find(key);
    if (it != nodeMap_.end()) {
      updateExistingNode(it->second, value);
      return;
    }
    addNewNode(key, value);
  }

  bool get(Key key, Value &value) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodeMap_.find(key);
    if (it != nodeMap_.end()) {
      moveToMostRecent(it->second);
      return true;
    }
    return false;
  }



private:
  int capacity_;
  NodeMap nodeMap_;
  std::mutex mutex_;
  NodePtr dummyHead_; // 虚拟头节点
  NodePtr dummyTail_;
};


}