#ifndef BAXTER_UTILS
#define BAXTER_UTILS

#include <memory>

template <typename T> using sptr = std::shared_ptr<T>;
template <typename T> using uptr = std::unique_ptr<T>;
#endif