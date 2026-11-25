#pragma once

#include <chrono>
#include <optional>

namespace capstone_fleet_runtime {

// Измерение сохраняет оба времени: ожидание независимого DDS /clock не омолаживает его.
template<class Message>
class TimedInput {
 public:
  using Clock = std::chrono::steady_clock;
  struct Sample { Message message; double stamp; Clock::time_point received; };
  explicit TimedInput(double timeout = 0.3) : timeout_(timeout) {}

  void receive(const Message &message, double stamp, double now, Clock::time_point wall) {
    advance(now, wall);
    const double age = now - stamp;
    if (age < 0) {
      if (age >= -timeout_ && !pending_) pending_ = Sample{message, stamp, wall};
    } else if (age <= timeout_) accept({message, stamp, wall});
  }
  void advance(double now, Clock::time_point wall) {
    if (previous_clock_ && now < *previous_clock_) {
      accepted_.reset();
      pending_.reset();
    }
    previous_clock_ = now;
    if (!pending_) return;
    const double age = now - pending_->stamp;
    if (seconds(wall - pending_->received) > timeout_ || age > timeout_ || age < -timeout_) {
      pending_.reset();
    } else if (age >= 0) {
      accept(*pending_);
      pending_.reset();
    }
  }
  bool fresh(double now, Clock::time_point wall) const {
    return accepted_ && now >= accepted_->stamp && now - accepted_->stamp <= timeout_ &&
           seconds(wall - accepted_->received) <= timeout_;
  }
  const std::optional<Sample> &sample() const { return accepted_; }

 private:
  static double seconds(Clock::duration duration) {
    return std::chrono::duration<double>(duration).count();
  }
  void accept(const Sample &sample) {
    if (!accepted_ || sample.stamp >= accepted_->stamp) accepted_ = sample;
  }
  double timeout_;
  std::optional<double> previous_clock_;
  std::optional<Sample> accepted_, pending_;
};
}  // namespace capstone_fleet_runtime
