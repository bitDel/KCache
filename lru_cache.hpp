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
      value = it->second->getValue();
      return true;
    }
    return false;
  }

  Value get(Key key) override {
    Value value{}; // 使用Value的默认构造函数
    get(key, value);
    return value;
  }
  
  void remove(Key key) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodeMap_.find(key);
    if (it != nodeMap_.end()) {
      removeNode(it->second);
      nodeMap_.erase(it);
    }
  }


private:

  void initalizeList() {
    // dummyHead_ = std::make_shared<LruNodeType>(Key(), Value());
    // dummyTail_ = std::make_shared<LruNodeType>(Key(), Value());
    // 显示构造临时临时对象，使用默认构造函数

    dummyHead_ = std::make_shared<LruNodeType>(Key{}, Value{});
    dummyTail_ = std::make_shared<LruNodeType>(Key{}, Value{});
    dummyHead_->next_ = dummyTail_;
    dummyTail_->prev_ = dummyHead_;
  }

  void updateExistingNode(NodePtr node, const Value &value) {
    node->setValue(value);
    moveToMostRecent(node);
  }

  void moveToMostRecent(NodePtr node) {
    removeNode(node);
    insertNode(node);
  }

  void removeNode(NodePtr node) {
    if (!node->prev_.expired() && node->next_) {
      auto prev = node->prev_.lock();
      prev->next_ = node->next_;
      node->next_->prev_ = prev;
      node->next_ = nullptr;
      // 清空指针，断开连接
    }
  }

  void insertNode(NodePtr node) {
    node->next_ = dummyTail_;
    node->prev_ = dummyTail_->prev_;
    dumyyTail_->prev_.lock()->next_ = node;
    dummyTail_->prev_ = node;
  }

  void evictLeastRecent() {
    NodePtr leastRecent = dummyHead_->next_;
    removeNode(leastRecent);
    nodeMap_.erase(leastRecent->getKey());
  }

private:
  int capacity_;
  NodeMap nodeMap_;
  std::mutex mutex_;
  NodePtr dummyHead_; // 虚拟头节点
  NodePtr dummyTail_;
};


}