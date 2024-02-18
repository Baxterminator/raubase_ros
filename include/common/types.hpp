#ifndef RAUBASE_TYPES
#define RAUBASE_TYPES

#include <memory>

namespace raubase {

// Pointers short name

template <typename T>
using sptr = std::shared_ptr<T>;

typedef unsigned long ulong;

}  // namespace raubase

#endif