#pragma once
#include <cstdint>
#include <cstddef>

inline uint64_t fnv1a64_init() {
  return 14695981039346656037ULL;
}

inline uint64_t fnv1a64_update(uint64_t h, const void* data, size_t n) {
  const uint8_t* p = (const uint8_t*)data;
  for (size_t i = 0; i < n; i++) {
    h ^= (uint64_t)p[i];
    h *= 1099511628211ULL;
  }
  return h;
}

inline uint64_t fnv1a64_update_u64(uint64_t h, uint64_t v) {
  return fnv1a64_update(h, &v, sizeof(v));
}
