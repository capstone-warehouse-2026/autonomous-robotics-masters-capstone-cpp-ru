#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>
#include <optional>
namespace capstone_system_test {
struct Point { double x, y; };
struct Rectangle { double xmin, ymin, xmax, ymax; };
inline Rectangle inflate(Rectangle rectangle, double radius) {
  return {rectangle.xmin - radius, rectangle.ymin - radius,
          rectangle.xmax + radius, rectangle.ymax + radius};
}
inline bool contains(Rectangle rectangle, Point point) {
  return point.x >= rectangle.xmin && point.x <= rectangle.xmax &&
         point.y >= rectangle.ymin && point.y <= rectangle.ymax;
}
// Пересечение всего отрезка, а не только концов: редкие точки не скрывают проезд через полку.
inline bool intersects(Rectangle rectangle, Point start, Point finish) {
  double enter = 0.0, leave = 1.0;
  for (int axis = 0; axis != 2; ++axis) {
    const double origin = axis == 0 ? start.x : start.y;
    const double delta = axis == 0 ? finish.x - start.x : finish.y - start.y;
    const double low = axis == 0 ? rectangle.xmin : rectangle.ymin;
    const double high = axis == 0 ? rectangle.xmax : rectangle.ymax;
    if (std::abs(delta) < 1e-12) {
      if (origin < low || origin > high) return false;
      continue;
    }
    double first = (low - origin) / delta, last = (high - origin) / delta;
    if (first > last) std::swap(first, last);
    enter = std::max(enter, first);
    leave = std::min(leave, last);
    if (enter > leave) return false;
  }
  return true;
}
inline double length(const std::vector<Point> &points) {
  double total = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  return total;
}
inline bool valid_detour(const std::vector<Point> &points, Rectangle obstacle) {
  if (points.size() < 3 || !intersects(obstacle, points.front(), points.back())) return false;
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (intersects(obstacle, points[i - 1], points[i])) return false;
  }
  return length(points) > std::hypot(points.back().x - points.front().x,
                                    points.back().y - points.front().y) + 0.2;
}
struct PositionSample { double time; Point position; };
inline std::optional<Point> nearest_position(const std::vector<PositionSample> &samples,
                                             double time, double tolerance) {
  std::optional<Point> result;
  double nearest_age = tolerance;
  for (const auto &sample : samples) {
    const double age = std::abs(sample.time - time);
    if (age <= nearest_age) { nearest_age = age; result = sample.position; }
  }
  return result;
}
inline std::vector<std::string> csv_row(const std::string &line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') { field += c; ++i; }
      else quoted = !quoted;
    } else if (c == ',' && !quoted) { fields.push_back(field); field.clear(); }
    else field += c;
  }
  if (quoted) throw std::invalid_argument("unterminated CSV field");
  fields.push_back(field);
  return fields;
}
}  // namespace capstone_system_test
