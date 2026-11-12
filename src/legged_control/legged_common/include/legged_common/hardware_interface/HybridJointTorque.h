#pragma once

#include <algorithm>
#include <cmath>

namespace legged {

struct HybridJointTorque {
  double position, velocity, feedforward, requested, applied;
};

// The actuator limit applies to the complete command, not each term separately.
inline HybridJointTorque computeHybridJointTorque(double qDes, double vDes, double kp, double kd, double feedforward,
                                                 double q, double v, double limit) {
  HybridJointTorque result;
  result.position = kp * (qDes - q);
  result.velocity = kd * (vDes - v);
  result.feedforward = feedforward;
  result.requested = result.position + result.velocity + result.feedforward;
  result.applied = std::isfinite(result.requested) && std::isfinite(limit) && limit > 0.0
                       ? std::max(-limit, std::min(limit, result.requested))
                       : 0.0;
  return result;
}

}  // namespace legged
