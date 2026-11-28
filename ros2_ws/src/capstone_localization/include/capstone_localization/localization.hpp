#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "capstone_localization/estimator.hpp"

namespace capstone_localization {

using Features = std::array<float, 7>;

struct LocalizationConfig {
  // Начальное состояние фильтра локализации.
  Pose initial = Pose::Zero();
  Pose fixed_variance = Pose(0.04 * 0.04, 0.04 * 0.04, 0.025 * 0.025);
  double search_radius = 0.22;
  double nis_limit = 16.27;  // 3 координаты, 0.001 уровень значимости.
  double wheel_timeout = 0.3;
};

struct Estimate {
  double stamp = 0.0;
  Filter odom;
  Filter global;
  double forward_velocity = 0.0;
  double yaw_rate = 0.0;
  bool imu_used = false;
};

struct ScanUpdate {
  std::optional<Match> match;
  Features features{};
  Pose measurement_variance = Pose::Zero();
  double nis = 0.0;
  bool accepted = false;
  bool fallback = false;
  std::string reason;
};

class CovarianceModel;

// Shared by the ROS node and the offline evaluator. Callers use timestamps in
// seconds, supply scans in base_link and interpolate GT only in the evaluator.
class Localization {
 public:
  explicit Localization(LocalizationConfig config = {}, bool learned = false, const std::string &model = "");
  ~Localization();
  void reset();
  void set_map(unsigned width, unsigned height, double resolution, const Pose &origin,
               const std::vector<std::int8_t> &data);
  bool has_map() const { return matcher_.ready(); }
  bool wheel(double stamp, double velocity, double wheel_yaw_rate, std::optional<double> imu_yaw_rate);
  ScanUpdate scan(double stamp, const Points &points);
  const std::optional<Estimate> &estimate() const { return estimate_; }
  const std::string &model_error() const { return model_error_; }

 private:
  struct Motion {
    double end;
    Pose increment;
    Covariance noise;
  };

  LocalizationConfig config_;
  bool learned_;
  std::unique_ptr<CovarianceModel> model_;
  std::string model_error_;
  ScanMatcher matcher_;
  std::optional<Estimate> estimate_;
  Filter anchor_;
  double anchor_stamp_ = 0.0;
  std::optional<double> last_scan_;
  Pose history_ = Pose::Zero();
  std::deque<Motion> motions_;
};

}  // namespace capstone_localization
