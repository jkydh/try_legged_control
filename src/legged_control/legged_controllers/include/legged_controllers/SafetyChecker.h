//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace legged {
using namespace ocs2;
using namespace centroidal_model;
class SafetyChecker {
 public:
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  bool check(const SystemObservation& observation, const vector_t& optimized_state, const vector_t& optimized_input) {
    if (!observation.state.allFinite() || !optimized_state.allFinite() || !optimized_input.allFinite()) {
      std::cerr << "[SafetyChecker] Non-finite state or policy!" << std::endl;
      return false;
    }
    return checkOrientation(observation);
  }

 protected:
  bool checkOrientation(const SystemObservation& observation) {
    vector_t pose = getBasePose(observation.state, info_);
    if (std::abs(pose(5)) > M_PI_2 || std::abs(pose(4)) > M_PI_2) {
      std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
      return false;
    }
    return true;
  }

  const CentroidalModelInfo& info_;
};

}  // namespace legged
