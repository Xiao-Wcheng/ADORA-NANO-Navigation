#pragma once

#include <cstddef>

namespace karto_dora {

class LocalizationHealth {
 public:
  LocalizationHealth(double minimum_response, std::size_t stop_after)
      : minimum_response_(minimum_response), stop_after_(stop_after) {}

  void ObserveRejected() noexcept { ++rejected_scans_; }
  void ObserveMatched(double response) noexcept {
    if (response < minimum_response_) {
      ++consecutive_losses_;
      ++loss_events_;
    } else {
      consecutive_losses_ = 0;
    }
  }
  std::size_t rejected_scans() const noexcept { return rejected_scans_; }
  std::size_t consecutive_losses() const noexcept { return consecutive_losses_; }
  std::size_t loss_events() const noexcept { return loss_events_; }
  bool safety_stop() const noexcept { return consecutive_losses_ >= stop_after_; }

 private:
  double minimum_response_;
  std::size_t stop_after_;
  std::size_t rejected_scans_{0};
  std::size_t consecutive_losses_{0};
  std::size_t loss_events_{0};
};

}  // namespace karto_dora
