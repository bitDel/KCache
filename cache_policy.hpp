#pragma once

namespace kcache {
    
template <typename Key, typename Value>
class CachePolicy {

public:
  virtual ~cache_policy() {};
  // 缓存接口
  virtual void put(Key key, Value value) = 0;

  // 访问的值以传出参数的形式返回
  virtual bool get(Key key, Value &value) = 0;

  virtual Value get(Key key) = 0;
};

}