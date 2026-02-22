#pragma once
#include <cmath>

struct Vec2 {
  double x = 0.0;
  double y = 0.0;

  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
  Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
  Vec2 operator*(double s) const { return Vec2(x * s, y * s); }

  Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
  Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

  double norm2() const { return x*x + y*y; }
  double norm()  const { return std::sqrt(norm2()); }
};

inline Vec2 operator*(double s, const Vec2& v) { return Vec2(v.x * s, v.y * s); }
