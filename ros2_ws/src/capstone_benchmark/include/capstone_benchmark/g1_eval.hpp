#ifndef CAPSTONE_G1_EVAL_HPP
#define CAPSTONE_G1_EVAL_HPP

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace capstone::g1 {
using Stamp = std::int64_t;
constexpr Stamp kSecond = 1000000000;
constexpr Stamp kGap = 40000000;
struct Pose { double x, y, yaw; };
using Stream = std::map<Stamp, Pose>;
using Trace = std::map<std::string, Stream>;
struct Coverage {
  std::size_t requested{}, exact{}, interpolated{}, outside{}, gap{};
  Stamp max_bracket{};
};
struct Match { Stamp stamp; Pose truth, estimate; };
struct Summary {
  double ate{}, p95{}, yaw_rmse{}, yaw_p95{};
  std::size_t rpe_n{}, loss_episodes{}, loss_censored{};
  std::optional<double> rpe_xy, rpe_yaw, recovery_mean_s;
  double longest_loss_s{};
};
Trace read_trace(std::istream& input, bool allow_legacy);
std::optional<Pose> sample(const Stream& s, Stamp stamp, Coverage* c = nullptr);
std::vector<Match> pair(const Stream& gt, const Stream& source, Stamp start, Coverage& c);
std::vector<Match> align_first(const std::vector<Match>& rows);
Summary summarize(const std::vector<Match>& rows);
}  // namespace capstone::g1
#endif
