// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include "roport/robot_interface.h"

namespace roport {

// NOLINTNEXTLINE(modernize-pass-by-value)
  Torques::Torques(const std::vector<double>& torques) noexcept : tau_J(torques) {}

  Torques::Torques(std::initializer_list<double> torques) {
    if (torques.size() != tau_J.size()) {
      throw std::invalid_argument("Invalid number of elements in tau_J.");
    }
    std::copy(torques.begin(), torques.end(), tau_J.begin());
  }

  // NOLINTNEXTLINE(modernize-pass-by-value)
  JointPositions::JointPositions(const std::vector<double>& joint_positions) noexcept
    : q(joint_positions) {}

  JointPositions::JointPositions(std::initializer_list<double> joint_positions) {
    if (joint_positions.size() != q.size()) {
      throw std::invalid_argument("Invalid number of elements in joint_positions.");
    }
    std::copy(joint_positions.begin(), joint_positions.end(), q.begin());
  }

// NOLINTNEXTLINE(modernize-pass-by-value)
  JointVelocities::JointVelocities(const std::vector<double>& joint_velocities) noexcept
    : dq(joint_velocities) {}

  JointVelocities::JointVelocities(std::initializer_list<double> joint_velocities) {
    if (joint_velocities.size() != dq.size()) {
      throw std::invalid_argument("Invalid number of elements in joint_velocities.");
    }
    std::copy(joint_velocities.begin(), joint_velocities.end(), dq.begin());
  }

}  // namespace end
