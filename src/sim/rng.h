#pragma once
#include <cstdint>
#include <cmath>

// Deterministic RNG: xorshift64* + Box-Muller for Gaussian
class Rng {
  uint64_t s;

  static uint64_t xorshift64star(uint64_t& x) {
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    return x * 2685821657736338717ULL;
  }

public:
  explicit Rng(uint64_t seed) : s(seed ? seed : 0x9e3779b97f4a7c15ULL) {}

  uint32_t u32() {
    uint64_t v = xorshift64star(s);
    return (uint32_t)(v >> 32);
  }

  double uniform01() {
    uint64_t v = xorshift64star(s);
    const uint64_t m = (v >> 11) | 1ULL;
    return (double)(m & ((1ULL<<53)-1)) / (double)(1ULL<<53);
  }

  double gaussian(double mean, double stddev) {
    double u1 = uniform01();
    double u2 = uniform01();
    if (u1 < 1e-12) u1 = 1e-12;
    double r = std::sqrt(-2.0 * std::log(u1));
    double th = 2.0 * 3.14159265358979323846 * u2;
    double z0 = r * std::cos(th);
    return mean + stddev * z0;
  }
};
