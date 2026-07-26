#include "karto_dora/localization_health.hpp"

#include <cstdlib>
#include <iostream>

void Require(bool value, const char *message) {
  if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

int main() {
  karto_dora::LocalizationHealth health(0.35, 5);
  for (int i = 0; i < 100; ++i) health.ObserveRejected();
  Require(health.loss_events() == 0 && !health.safety_stop(),
          "ordinary OpenKarto rejection was classified as localization loss");
  for (int i = 0; i < 4; ++i) health.ObserveMatched(0.20);
  Require(health.consecutive_losses() == 4 && !health.safety_stop(),
          "low-confidence hysteresis triggered too early");
  health.ObserveMatched(0.80);
  Require(health.consecutive_losses() == 0,
          "good match did not clear consecutive losses");
  for (int i = 0; i < 5; ++i) health.ObserveMatched(0.20);
  Require(health.safety_stop() && health.loss_events() == 9,
          "sustained low-confidence matches did not trigger safety stop");
  std::cout << "localization_health_test PASS\n";
}
