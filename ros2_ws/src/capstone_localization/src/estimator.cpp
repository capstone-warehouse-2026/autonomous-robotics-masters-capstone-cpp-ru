#include "capstone_localization/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace capstone_localization {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

Eigen::Matrix2d rotation(double angle) {
  return Eigen::Rotation2Dd(angle).toRotationMatrix();
}

Pose innovation(const Pose &measurement, const Pose &prediction) {
  Pose result = measurement - prediction;
  result.z() = wrap_angle(result.z());
  return result;
}

}  // namespace

double wrap_angle(double radians) { return std::remainder(radians, kTwoPi); }

Pose compose(const Pose &parent_from_frame, const Pose &frame_from_child) {
  Pose result;
  result.head<2>() = parent_from_frame.head<2>() + rotation(parent_from_frame.z()) * frame_from_child.head<2>();
  result.z() = wrap_angle(parent_from_frame.z() + frame_from_child.z());
  return result;
}

Pose between(const Pose &world_from_parent, const Pose &world_from_child) {
  Pose result;
  result.head<2>() = rotation(-world_from_parent.z()) *
      (world_from_child.head<2>() - world_from_parent.head<2>());
  result.z() = wrap_angle(world_from_child.z() - world_from_parent.z());
  return result;
}

void Filter::predict(const Pose &increment, const Covariance &process_noise) {
  const Eigen::Matrix2d axes = rotation(pose.z()); // Получаем матрицу поворота.
  const Eigen::Vector2d displacement = axes * increment.head<2>(); // Преобразуем в глобальные координаты.
  Covariance state_jacobian = Covariance::Identity(); // Якобиан состояния.
  state_jacobian(0, 2) = -displacement.y();
  state_jacobian(1, 2) = displacement.x();
  Covariance noise_jacobian = Covariance::Identity(); // Якобиан шума процесса.
  noise_jacobian.topLeftCorner<2, 2>() = axes;
  pose = compose(pose, increment);
  covariance = state_jacobian * covariance * state_jacobian.transpose() +
      noise_jacobian * process_noise * noise_jacobian.transpose(); // Обновляем ковариацию состояния с учетом шума процесса.
  covariance = (0.5 * (covariance + covariance.transpose())).eval(); // Симметризация ковариационной матрицы.
}

double Filter::nis(const Pose &measurement, const Covariance &measurement_noise) const {
  // Оценка, насколько новое измерение согласуется с прогнозом фильтра.
  const Pose delta = innovation(measurement, pose);
  const Covariance residual_covariance = covariance + measurement_noise;
  return delta.dot(residual_covariance.ldlt().solve(delta)); // Вычисление NIS.
}

void Filter::correct(const Pose &measurement, const Covariance &measurement_noise) {
  // Объединение прогноза фильтра с измерением от LiDAR и map.
  const Covariance residual_covariance = covariance + measurement_noise;
  const Covariance gain = residual_covariance.ldlt().solve(covariance.transpose()).transpose(); // Вычисление коэффициента усиления Калмана.
  pose += gain * innovation(measurement, pose); // Обновление состояния фильтра с учетом измерения.
  pose.z() = wrap_angle(pose.z());
  const Covariance remaining = Covariance::Identity() - gain;
  covariance = remaining * covariance * remaining.transpose() +
      gain * measurement_noise * gain.transpose(); // Форма Джозефа.
  covariance = (0.5 * (covariance + covariance.transpose())).eval();
}

void ScanMatcher::set_map(unsigned width,
                          unsigned height,
                          double resolution,
                          const Pose &origin,
                          const std::vector<signed char> &data) {
  if (width == 0 || height == 0 || data.size() != static_cast<std::size_t>(width) * height ||
      !std::isfinite(resolution) || resolution <= 0.0 || !origin.allFinite()) {
        // Проверка валидности входных данных.
    throw std::invalid_argument("invalid occupancy grid");
  }
  std::vector<std::vector<Surface>> surfaces(data.size()); // Создание вектора поверхностей для каждой ячейки карты.
  const int faces[][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * width + x; // Индекс текущей ячейки в массиве данных карты.
      if (data[index] < 65) {
        continue;
      }
      for (const auto &face : faces) {
        const int nx = static_cast<int>(x) + face[0], ny = static_cast<int>(y) + face[1];
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) {
          continue;
        }
        const auto neighbor = data[static_cast<std::size_t>(ny) * width + nx];
        if (neighbor >= 0 && neighbor <= 25) {
          const Eigen::Vector2d normal(face[0], face[1]);
          const Eigen::Vector2d center = resolution * (Eigen::Vector2d(x + 0.5, y + 0.5) + 0.5 * normal);
          surfaces[index].push_back({center, normal}); // Добавление поверхности в список поверхностей текущей ячейки.
        }
      }
    }
  }
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_ = origin;
  surfaces_ = std::move(surfaces);
}

const ScanMatcher::Surface *ScanMatcher::nearest(const Eigen::Vector2d &point, double radius) const {
  // Находит ближайшую поверхность к заданной точке в пределах указанного радиуса.
  const Surface *best = nullptr;
  double best_squared = radius * radius;
  const double pad = radius + 0.5 * resolution_;
  if (point.x() < -pad || point.y() < -pad ||
      point.x() > width_ * resolution_ + pad || point.y() > height_ * resolution_ + pad) {
    return nullptr;
  }
  const int first_x = std::max(0, static_cast<int>(std::floor((point.x() - pad) / resolution_)));
  const int first_y = std::max(0, static_cast<int>(std::floor((point.y() - pad) / resolution_)));
  const int last_x = std::min(static_cast<int>(width_) - 1,
      static_cast<int>(std::floor((point.x() + pad) / resolution_)));
  const int last_y = std::min(static_cast<int>(height_) - 1,
      static_cast<int>(std::floor((point.y() + pad) / resolution_)));
      // Итерируемся по ячейкам карты в пределах радиуса поиска и ищем ближайшую поверхность к заданной точке.
  for (int y = first_y; y <= last_y; ++y) {
    for (int x = first_x; x <= last_x; ++x) {
      for (const auto &surface : surfaces_[static_cast<std::size_t>(y) * width_ + x]) {
        const Eigen::Vector2d displacement = point - surface.center;
        const double perpendicular = displacement.dot(surface.normal);
        const double lateral = std::max(0.0, std::abs(displacement.x() * surface.normal.y() -
            displacement.y() * surface.normal.x()) - 0.5 * resolution_);
        const double squared = perpendicular * perpendicular + lateral * lateral;
        if (squared < best_squared) {
          best_squared = squared;
          best = &surface;
        }
      }
    }
  }
  return best;
}

std::optional<Match> ScanMatcher::match(const Points &scan, const Pose &prediction, double search_radius) const {
  constexpr int kIterations = 12;
  constexpr std::size_t kMinimumInliers = 12;
  constexpr double kHuberScale = 0.08;
  if (!ready() || scan.size() < kMinimumInliers || !prediction.allFinite()) {
    return std::nullopt;
  }
  Pose estimate = between(origin_, prediction);
  Match result;
  for (int iteration = 0; iteration <= kIterations; ++iteration) {
    Covariance information = Covariance::Zero();
    Pose gradient = Pose::Zero();
    std::size_t inliers = 0;
    double squared = 0.0;
    const Eigen::Matrix2d axes = rotation(estimate.z());
    for (const auto &point : scan) {
      if (!point.allFinite()) {
        continue;
      }
      const Eigen::Vector2d lever = axes * point;
      const Eigen::Vector2d map_point = estimate.head<2>() + lever;
      const auto *surface = nearest(map_point, search_radius);
      if (!surface) {
        continue;
      }
      const double residual = surface->normal.dot(map_point - surface->center);
      const Pose jacobian(surface->normal.x(), surface->normal.y(),
          surface->normal.dot(Eigen::Vector2d(-lever.y(), lever.x())));
      const double weight = std::abs(residual) <= kHuberScale ? 1.0 : kHuberScale / std::abs(residual);
      information.noalias() += weight * jacobian * jacobian.transpose();
      gradient.noalias() += weight * jacobian * residual;
      squared += residual * residual;
      ++inliers;
    }
    if (inliers < kMinimumInliers || 4 * inliers < scan.size()) {
      return std::nullopt;
    }
    // Use a scale-independent geometry measure. Reject completely unobservable
    // corridors instead of injecting the initial guess as a confident measurement.
    const auto diagonal = information.diagonal().cwiseMax(1e-12).cwiseSqrt().cwiseInverse().eval();
    const Covariance normalized = diagonal.asDiagonal() * information * diagonal.asDiagonal();
    const Eigen::SelfAdjointEigenSolver<Covariance> eigen(normalized, Eigen::EigenvaluesOnly);
    if (eigen.info() != Eigen::Success) {
      return std::nullopt;
    }
    result.geometry_ratio = eigen.eigenvalues().minCoeff() / eigen.eigenvalues().maxCoeff();
    if (!std::isfinite(result.geometry_ratio) || result.geometry_ratio < 1e-5) {
      return std::nullopt;
    }
    const Pose step = -information.ldlt().solve(gradient);
    if (!step.allFinite()) {
      return std::nullopt;
    }
    if (iteration == kIterations || (step.head<2>().norm() < 1e-4 && std::abs(step.z()) < 1e-4)) {
      result.pose = compose(origin_, estimate);
      result.inlier_count = static_cast<double>(inliers);
      result.inlier_fraction = result.inlier_count / scan.size();
      result.rms = std::sqrt(squared / inliers);
      return result;
    }
    // Limit a single linearization step when the seed lies near an aisle corner.
    const double scale = std::min({1.0, 0.15 / std::max(1e-12, step.head<2>().norm()),
        0.15 / std::max(1e-12, std::abs(step.z()))});
    estimate += scale * step;
    estimate.z() = wrap_angle(estimate.z());
  }
  return std::nullopt;
}

}  // namespace capstone_localization
