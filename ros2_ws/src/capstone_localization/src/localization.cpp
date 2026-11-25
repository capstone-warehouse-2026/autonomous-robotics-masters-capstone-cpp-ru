#include "capstone_localization/localization.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace capstone_localization {
// Versioned, fixed-size 7 -> 8 -> 3 MLP. Training must use the same raw features.
class CovarianceModel {
   public:
    explicit CovarianceModel(const std::string &path) {
        std::ifstream in(path);
        std::string tag;
        if (!(in >> tag) || tag != "G1_MLP_V1")
            throw std::runtime_error("missing/invalid G1_MLP_V1 model");
        for (double &v : weights_)
            if (!(in >> v) || !std::isfinite(v))
                throw std::runtime_error("invalid model weights");
        if (in >> tag)
            throw std::runtime_error("unexpected trailing model data");
    }
    Pose variance(const Features &f) const {
        double h[8];
        size_t k = 0;
        for (double &v : h) {
            v = 0;
            for (float x : f)
                v += weights_[k++] * x;
            v = std::tanh(v + weights_[k++]);
        }
        Pose out;
        for (int i = 0; i < 3; ++i) {
            double z = 0;
            for (double v : h)
                z += weights_[k++] * v;
            z += weights_[k++];
            if (!std::isfinite(z))
                throw std::runtime_error("non-finite inference");
            out[i] = std::clamp(std::max(z, 0.0) + std::log1p(std::exp(-std::abs(z))), 1e-5,
                                i == 2 ? 0.25 : 1.0);
        }
        return out;
    }

   private:
    std::array<double, 91> weights_{};
};
Localization::Localization(LocalizationConfig config, bool learned, const std::string &model)
    : config_(config), learned_(learned) {
    if (!config_.initial.allFinite() || !config_.fixed_variance.allFinite() ||
        (config_.fixed_variance.array() <= 0).any() || !std::isfinite(config_.wheel_timeout) ||
        config_.wheel_timeout <= 0 || !std::isfinite(config_.search_radius) ||
        config_.search_radius <= 0 || !std::isfinite(config_.nis_limit) || config_.nis_limit <= 0)
        throw std::invalid_argument("invalid localization configuration");
    for (int i = 0; i < 3; ++i)
        config_.fixed_variance[i] =
            std::clamp(config_.fixed_variance[i], 1e-5, i == 2 ? 0.25 : 1.0);
    if (learned_)
        try {
            model_ = std::make_unique<CovarianceModel>(model);
        } catch (const std::exception &e) {
            model_error_ = e.what();
        }
}
Localization::~Localization() = default;
void Localization::reset() {
    estimate_.reset();
    last_scan_.reset();
    motions_.clear();
    history_.setZero();
    anchor_ = Filter{};
    anchor_stamp_ = 0;
}
void Localization::set_map(unsigned w, unsigned h, double r, const Pose &origin,
                           const std::vector<std::int8_t> &data) {
    matcher_.set_map(w, h, r, origin, data);
}
bool Localization::wheel(double stamp, double velocity, double yaw, std::optional<double> imu) {
    if (!std::isfinite(stamp) || !std::isfinite(velocity) || !std::isfinite(yaw))
        return false;
    if (imu && !std::isfinite(*imu))
        imu.reset();
    // Bag/simulation clock rewinds start a new independent run.
    if (estimate_ && stamp < estimate_->stamp - 0.5)
        reset();
    if (!estimate_) {
        Estimate e;
        e.stamp = stamp;
        e.global.pose = config_.initial;
        e.global.covariance = config_.fixed_variance.asDiagonal();
        e.odom.covariance = Covariance::Identity() * 1e-6;
        e.forward_velocity = velocity;
        e.yaw_rate = imu.value_or(yaw);
        e.imu_used = imu.has_value();
        estimate_ = e;
        return true;
    }
    const double dt = stamp - estimate_->stamp;
    if (dt <= 0)
        return false;
    estimate_->stamp = stamp;
    estimate_->forward_velocity = velocity;
    estimate_->yaw_rate = imu.value_or(yaw);
    estimate_->imu_used = imu.has_value();
    if (dt > config_.wheel_timeout) {
        // Missing measurements are not a license to extrapolate through a dropout.
        estimate_->global.covariance += Covariance::Identity() * dt;
        estimate_->odom.covariance += Covariance::Identity() * dt;
        return true;
    }
    const double angle = estimate_->yaw_rate * dt;
    const double sinc = std::abs(angle) < 1e-8 ? 1 : std::sin(angle) / angle;
    const double cosc = std::abs(angle) < 1e-8 ? angle / 2 : (1 - std::cos(angle)) / angle;
    Pose step(velocity * dt * sinc, velocity * dt * cosc, angle);
    Covariance q =
        Pose(0.0025 * dt + 0.01 * std::abs(step.x()), 0.0025 * dt, (imu ? 0.0004 : 0.004) * dt)
            .asDiagonal();
    estimate_->odom.predict(step, q);
    estimate_->global.predict(step, q);
    return true;
}
ScanUpdate Localization::scan(double stamp, const Points &points) {
    ScanUpdate out;
    out.measurement_variance = config_.fixed_variance;
    out.fallback = learned_ && !model_;
    if (!std::isfinite(stamp) || !estimate_) {
        out.reason = "no_odometry";
        return out;
    }
    // Do not apply a delayed scan at the wrong pose. Replay/interpolation is intentionally not
    // implicit.
    if (stamp < estimate_->stamp - 0.05 || stamp > estimate_->stamp + 0.05 ||
        (last_scan_ && stamp <= *last_scan_)) {
        out.reason = "stale_or_unsynchronized_scan";
        return out;
    }
    if (!has_map()) {
        out.reason = "no_map";
        return out;
    }
    last_scan_ = stamp;
    Points finite;
    for (const auto &p : points)
        if (p.allFinite())
            finite.push_back(p);
    out.match = matcher_.match(finite, estimate_->global.pose, config_.search_radius);
    if (!out.match) {
        out.reason = "scan_match_failed";
        return out;
    }
    const auto &m = *out.match;
    Pose innovation = m.pose - estimate_->global.pose;
    innovation.z() = wrap_angle(innovation.z());
    out.features = {float(m.rms),
                    float(m.inlier_fraction),
                    float(m.inlier_count),
                    float(m.geometry_ratio),
                    float(innovation.head<2>().norm()),
                    float(std::abs(innovation.z())),
                    float(history_.norm())};
    if (model_)
        try {
            out.measurement_variance = model_->variance(out.features);
        } catch (const std::exception &) {
            out.fallback = true;
        }
    const Covariance r = out.measurement_variance.asDiagonal();
    out.nis = estimate_->global.nis(m.pose, r);
    if (!std::isfinite(out.nis) || out.nis > config_.nis_limit) {
        out.reason = "innovation_gate";
        return out;
    }
    estimate_->global.correct(m.pose, r);
    history_ = innovation;
    out.accepted = true;
    out.reason = out.fallback ? "accepted_fixed_fallback" : "accepted";
    return out;
}
}  // namespace capstone_localization
