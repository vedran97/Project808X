/**
 * @file InverseKinematics.hpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @author Vedant Ranade (vedantr1@umd.edu)
 * @author Aaqib Barodawala (abarodaw@umd.edu)
 * @brief InverseKinematics Class
 * @version 0.1
 * @date 2023-10-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef InverseKinematics_HPP
#define InverseKinematics_HPP

#include <algorithm>
#include <array>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>

#include "ForwardKinematics.hpp"

using namespace Eigen;

namespace a3c {
using JointAngles = std::array<double, 6>;

/**
 * @brief Class IK
 *
 */
class InverseKinematics {
 private:
  float deltaTimeSecs;
  JointAngles initialJointAngles;


 public:
  explicit InverseKinematics(const JointAngles& inInitialJointAngles) noexcept;
  std::vector <JointAngles> linearIK(const Pose& currentPose, const Pose& targetPose);
  MatrixXd getJacobian(const JointAngles &jointAngles);
};
}  // namespace a3c

#endif