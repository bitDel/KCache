#pragma once

#include "arc_cache_node.hpp"

#include <unordered_map>
#include <mutex>

namespace kcache {

template<typename Key, typename Value>
class ArcLruPart {

public:
  using NodeType = ArcNode;
  using NodePtr = std::shared_ptr<NodeType>;
  using NodeMap = std::unordered_map<Key, NodePtr>;

  explicit ArcLruPart(size_t capacity, size_t transformThreshold)
    : capacity_(capacity),
      ghostCapacity_(capacity),
      transformThreshold_(transformThreshold)
  {
    initializeLists();
  }

  bool put(Key key, Value value) {
    if (capacity_ == 0) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = mainCache_.find(key);
    if (it != mainCache_.end()) {
      return updateExi
    }
  }



  

private:
  void initializeList() {
    mainHead_ = std::make_shared<NodeType>();
    mainTail_ = std::make_shared<NodeType>();
    mainHead_->next_ = mainTail_;
    mainTail_->prev_ = mainHead_;
    
    ghostHead_ = std::make_shared<NodeType>();
    ghostHead_ = std::make_shared<NodeType>();
    ghostHead_->next_ = ghostTail_;
    ghostTail_->prev_ = ghostHead_;
  }

  void removeOldestGhost() {
    NodePtr oldestGhost = ghostTail_->prev_->prev_.lock();
    if (!oldestGhost || oldestGhost == ghostHead_) {
      return;
    }
    removeFromGhost(oldestGhost);
    ghostCache_.erase(oldestGhost->getKey());
  }

  void removeFromMain(NodePtr node) {
    if (!node->prev_.expired() && node->next_) {
      auto prev = node->prev_.lock();
      prev->next_ = node->next_;
      node->next_->prev_ = node->prev_;
      node->next_ = nullptr;
    }
  }

  void removeFromGhost(NodePtr node) {
    if (!node->prev_.expired() && node->next_) {
      auto prev = node->prev_.lock();
      prev->next_ = node->next_;
      node->next_->prev_ = node->prev_;
      node->next_ = nullptr;
    }
  }

  void addToGhost(NodePtr node) {
    node->accessCount_ = 1;
    node->next_ = ghostHead_->next_;
    node->prev_ = ghostHead_;
    ghostHead_->next_->prev_ = node;
    ghostHead_->next_ = node;

    ghostCache_[node->getKey()] = node;
  }













private:
  size_t capacity_;
  size_t ghostCapacity_;
  // 转换阈值
  size_t transformThreshold_;
  std::mutex mutex_;

  NodeMap mainCache_;
  NodeMap ghostCache_;

  NodePtr mainHead_;
  NodePtr mainTail_;

  NodePtr ghostHead_;
  NodePtr ghostTail_;

};

}
