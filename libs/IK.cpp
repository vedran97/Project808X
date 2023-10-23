/**
 * @file IK.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief
 * @version 0.1
 * @date 2023-10-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <cmath>
#include <iostream>

#include "InverseKinematics.hpp"

namespace a3c {
/**
 * @brief Construct a new Inverse Kinematics:: Inverse Kinematics object
 *
 * @param FM
 */
InverseKinematics::InverseKinematics(const Eigen::Matrix4d& FM) noexcept {}

/**
 * @brief Solve for Inverse Kinematics
 *
 * @param currentAngles
 * @param targetAngles
 * @return std::vector <JointAngles>
 */
std::vector<JointAngles> ik(const JointAngles& currentAngles,
                            const JointAngles& targetAngles) {
  // TODO: add implementation
  return {};
}
}  // namespace a3c