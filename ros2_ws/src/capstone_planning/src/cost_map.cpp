#include "capstone_planning/cost_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace capstone_planning {

namespace {

constexpr double kNoSource = 1e20;

// 1D squared distance transform of a sampled function f (lower envelope of parabolas).
void distance_transform_1d(const std::vector<double> &f, std::vector<double> &d, std::vector<int> &v,
                           std::vector<double> &z, int n) {
  const auto intersection = [&f](int q, int p) {
    return ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) / (2.0 * (q - p));
  };
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < n; ++q) {
    double s = intersection(q, v[k]);
    while (s <= z[k]) {  // terminates at k == 0 because z[0] = -inf
      --k;
      s = intersection(q, v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const double dq = q - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

double proximity(double clearance, double inflation_radius, double band) {
  if (band <= 0.0) {
    return 0.0;
  }
  const double t = 1.0 - (clearance - inflation_radius) / band;
  return t <= 0.0 ? 0.0 : std::min(1.0, t) * std::min(1.0, t);
}

void require(bool ok, const std::string &what) {
  if (!ok) {
    throw std::invalid_argument("CostParams: " + what);
  }
}

}  // namespace

void CostParams::validate() const {
  const auto nonneg = [](double v) { return std::isfinite(v) && v >= 0.0; };
  require(std::isfinite(robot_radius) && robot_radius > 0.0, "robot_radius must be > 0");
  require(nonneg(inflation_margin), "inflation_margin must be >= 0");
  require(nonneg(clearance_band) && nonneg(clearance_weight), "clearance_band/weight must be >= 0");
  require(nonneg(observed_band) && nonneg(observed_weight), "observed_band/weight must be >= 0");
  require(nonneg(unknown_weight), "unknown_weight must be >= 0");
}

std::vector<double> squared_distance_transform(const std::vector<std::uint8_t> &source, int width, int height) {
  if (width <= 0 || height <= 0 || source.size() != static_cast<std::size_t>(width) * height) {
    throw std::invalid_argument("squared_distance_transform: size mismatch");
  }
  std::vector<double> out(source.size());
  for (std::size_t i = 0; i < source.size(); ++i) {
    out[i] = source[i] ? 0.0 : kNoSource;
  }
  const int n = std::max(width, height);
  std::vector<double> f(n), d(n), z(n + 1);
  std::vector<int> v(n);
  // Columns, then rows.
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      f[y] = out[static_cast<std::size_t>(y) * width + x];
    }
    distance_transform_1d(f, d, v, z, height);
    for (int y = 0; y < height; ++y) {
      out[static_cast<std::size_t>(y) * width + x] = d[y];
    }
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      f[x] = out[static_cast<std::size_t>(y) * width + x];
    }
    distance_transform_1d(f, d, v, z, width);
    for (int x = 0; x < width; ++x) {
      out[static_cast<std::size_t>(y) * width + x] = d[x];
    }
  }
  return out;
}

CostMap::CostMap(const FusedMap &map, const CostParams &params, const std::vector<double> *extra_cost)
    : grid_(map.grid), params_(params) {
  if (map.observed_obstacle.size() != map.grid.size()) {
    throw std::invalid_argument("CostMap: observed_obstacle size mismatch");
  }
  if (extra_cost != nullptr) {
    if (extra_cost->size() != map.grid.size()) {
      throw std::invalid_argument("CostMap: extra_cost size mismatch");
    }
    for (double v : *extra_cost) {
      if (!std::isfinite(v) || v < 0.0) throw std::invalid_argument("CostMap: extra_cost must be finite and >= 0");
    }
  }
  build(map.observed_obstacle, extra_cost);
}

CostMap::CostMap(const Grid &map, const CostParams &params) : grid_(map), params_(params) {
  build(std::vector<std::uint8_t>(map.size(), 0), nullptr);
}

void CostMap::build(const std::vector<std::uint8_t> &observed_obstacle, const std::vector<double> *extra_cost) {
  if (!grid_.valid()) {
    throw std::invalid_argument("CostMap: invalid grid");
  }
  params_.validate();
  const std::size_t n = grid_.size();
  obstacle_.assign(n, 0);
  bool any_observed = false;
  for (std::size_t i = 0; i < n; ++i) {
    const std::int8_t v = grid_.data[i];
    obstacle_[i] = (is_occupied_value(v) || (params_.unknown_is_obstacle && is_unknown_value(v))) ? 1 : 0;
    any_observed = any_observed || observed_obstacle[i] != 0;
  }
  const double res = grid_.resolution;
  const auto to_clearance = [res](double squared_cells) {
    return squared_cells >= kNoSource ? std::numeric_limits<double>::infinity()
                                      : std::max(0.0, std::sqrt(squared_cells) * res - 0.5 * res);
  };
  const std::vector<double> d2 = squared_distance_transform(obstacle_, grid_.width, grid_.height);
  std::vector<double> observed_d2;
  if (any_observed && params_.observed_weight > 0.0) {
    observed_d2 = squared_distance_transform(observed_obstacle, grid_.width, grid_.height);
  }

  const double r = params_.inflation_radius();
  clearance_.resize(n);
  cost_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    clearance_[i] = obstacle_[i] ? 0.0 : to_clearance(d2[i]);
    if (obstacle_[i] || clearance_[i] < r) {
      cost_[i] = kBlocked;
      continue;
    }
    double c = kMinCostPerMeter + params_.clearance_weight * proximity(clearance_[i], r, params_.clearance_band);
    if (!observed_d2.empty()) {
      c += params_.observed_weight * proximity(to_clearance(observed_d2[i]), r, params_.observed_band);
    }
    if (is_unknown_value(grid_.data[i])) {
      c += params_.unknown_weight;
    }
    if (extra_cost != nullptr) {
      c += (*extra_cost)[i];
    }
    cost_[i] = c;
  }
}

}  // namespace capstone_planning
