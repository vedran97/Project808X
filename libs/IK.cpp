/**
 * @file IK.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @author Vedant Ranade (vedantr1@umd.edu)
 * @author Aaqib Barodawala (abarodaw@umd.edu)
 * @brief Inverse Kinematics class implementation
 * @version 0.1
 * @date 2023-10-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <cmath>
#include <iostream>

#include "include/ForwardKinematics.hpp"
#include "include/InverseKinematics.hpp"

namespace a3c {
/**
 * @brief Construct a new Inverse Kinematics:: Inverse Kinematics object
 *
 * @param FM
 */
InverseKinematics::InverseKinematics(
    const JointAngles& inInitialJointAngles) noexcept
    : deltaTimeSecs(1) {
  this->initialJointAngles = inInitialJointAngles;
}

/**
 * @brief Solve for Inverse Kinematics
 * @note This function takes in currentPose, targetPose and gives a vector of
 * jointAngles which result in a linear translation between currentPose and
 * TargetPose
 * @param currentAngles
 * @param targetAngles
 * @return std::vector <JointAngles>
 */
std::vector<JointAngles> InverseKinematics::linearIK(const Pose& currentPose,
                                                     const Pose& targetPose) {
  std::vector<JointAngles> jointTrajectory;
  auto phi = 0;
  auto dPhi_dt = 1;
  auto& currentPosition = currentPose.position;
  auto& targetPosition = targetPose.position;
  auto currentAngles = initialJointAngles;
  auto getJointAnglesFromVector = [](const auto& vec) {
    JointAngles ja = {vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]};
    return ja;
  };
  for (;;) {
    if (phi >= 1) {
      break;
    }
    auto newPosition = (1 - phi) * currentPosition + phi * targetPosition;
    auto dPos_dt = (newPosition - currentPosition) / deltaTimeSecs;
    auto joint_vels = getJacobian(currentAngles)
                          .completeOrthogonalDecomposition()
                          .pseudoInverse() *
                      dPos_dt;
    JointAngles newJA;
    auto jointvel_into_delta_t =
        getJointAnglesFromVector(joint_vels * deltaTimeSecs);
    for (int i = 0; i < 6; i++) {
      newJA[i] = currentAngles[i] + jointvel_into_delta_t[i];
    }
    jointTrajectory.push_back(newJA);
    currentAngles = newJA;
    phi += dPhi_dt * deltaTimeSecs;
  }
  jointTrajectory.shrink_to_fit();
  return jointTrajectory;
}

MatrixXd InverseKinematics::getJacobian(const JointAngles& jointAngles) {
  const auto jacobianSize = 6;
  MatrixXd J(jacobianSize, jacobianSize);
  auto theta1 = jointAngles.at(0);
  auto theta2 = jointAngles.at(1);
  auto theta3 = jointAngles.at(2);
  auto theta4 = jointAngles.at(3);
  auto theta5 = jointAngles.at(4);
  J.row(0) << -0.23000000000000001 * sin(theta1) * sin(theta2) +
                  0.16350000000000001 * sin(theta1) * sin(theta4) *
                      cos(theta2 + theta3) -
                  0.16600000000000001 * sin(theta1) * sin(theta5) *
                      cos(theta4) * cos(theta2 + theta3) -
                  0.16600000000000001 * sin(theta1) * sin(theta2 + theta3) *
                      cos(theta5) -
                  0.23000000000000001 * sin(theta1) * sin(theta2 + theta3) -
                  0.16600000000000001 * sin(theta4) * sin(theta5) *
                      cos(theta1) -
                  0.16350000000000001 * cos(theta1) * cos(theta4) +
                  0.00050000000000000001 * cos(theta1),
      0.16350000000000001 * sin(theta4) * sin(theta2 + theta3) * cos(theta1) -
          0.16600000000000001 * sin(theta5) * sin(theta2 + theta3) *
              cos(theta1) * cos(theta4) +
          0.23000000000000001 * cos(theta1) * cos(theta2) +
          0.16600000000000001 * cos(theta1) * cos(theta5) *
              cos(theta2 + theta3) +
          0.23000000000000001 * cos(theta1) * cos(theta2 + theta3),
      0.16350000000000001 * sin(theta4) * sin(theta2 + theta3) * cos(theta1) -
          0.16600000000000001 * sin(theta5) * sin(theta2 + theta3) *
              cos(theta1) * cos(theta4) +
          0.16600000000000001 * cos(theta1) * cos(theta5) *
              cos(theta2 + theta3) +
          0.23000000000000001 * cos(theta1) * cos(theta2 + theta3),
      0.16350000000000001 * sin(theta1) * sin(theta4) -
          0.16600000000000001 * sin(theta1) * sin(theta5) * cos(theta4) -
          0.16600000000000001 * sin(theta4) * sin(theta5) * cos(theta1) *
              cos(theta2 + theta3) -
          0.16350000000000001 * cos(theta1) * cos(theta4) *
              cos(theta2 + theta3),
      -0.16600000000000001 * sin(theta1) * sin(theta4) * cos(theta5) -
          0.16600000000000001 * sin(theta5) * sin(theta2 + theta3) *
              cos(theta1) +
          0.16600000000000001 * cos(theta1) * cos(theta4) * cos(theta5) *
              cos(theta2 + theta3),  // cppcheck-suppress constStatement
      0;

  J.row(1) << -0.16600000000000001 * sin(theta1) * sin(theta4) * sin(theta5) -
                  0.16350000000000001 * sin(theta1) * cos(theta4) +
                  0.00050000000000000001 * sin(theta1) +
                  0.23000000000000001 * sin(theta2) * cos(theta1) -
                  0.16350000000000001 * sin(theta4) * cos(theta1) *
                      cos(theta2 + theta3) +
                  0.16600000000000001 * sin(theta5) * cos(theta1) *
                      cos(theta4) * cos(theta2 + theta3) +
                  0.16600000000000001 * sin(theta2 + theta3) * cos(theta1) *
                      cos(theta5) +
                  0.23000000000000001 * sin(theta2 + theta3) * cos(theta1),
      0.16350000000000001 * sin(theta1) * sin(theta4) * sin(theta2 + theta3) -
          0.16600000000000001 * sin(theta1) * sin(theta5) *
              sin(theta2 + theta3) * cos(theta4) +
          0.23000000000000001 * sin(theta1) * cos(theta2) +
          0.16600000000000001 * sin(theta1) * cos(theta5) *
              cos(theta2 + theta3) +
          0.23000000000000001 * sin(theta1) * cos(theta2 + theta3),
      0.16350000000000001 * sin(theta1) * sin(theta4) * sin(theta2 + theta3) -
          0.16600000000000001 * sin(theta1) * sin(theta5) *
              sin(theta2 + theta3) * cos(theta4) +
          0.16600000000000001 * sin(theta1) * cos(theta5) *
              cos(theta2 + theta3) +
          0.23000000000000001 * sin(theta1) * cos(theta2 + theta3),
      -0.16600000000000001 * sin(theta1) * sin(theta4) * sin(theta5) *
              cos(theta2 + theta3) -
          0.16350000000000001 * sin(theta1) * cos(theta4) *
              cos(theta2 + theta3) -
          0.16350000000000001 * sin(theta4) * cos(theta1) +
          0.16600000000000001 * sin(theta5) * cos(theta1) * cos(theta4),
      -0.16600000000000001 * sin(theta1) * sin(theta5) * sin(theta2 + theta3) +
          0.16600000000000001 * sin(theta1) * cos(theta4) * cos(theta5) *
              cos(theta2 + theta3) +
          0.16600000000000001 * sin(theta4) * cos(theta1) *
              cos(theta5),  // cppcheck-suppress constStatement
      0;

  J.row(2) << 0,
      -0.23000000000000001 * sin(theta2) +
          0.16350000000000001 * sin(theta4) * cos(theta2 + theta3) -
          0.16600000000000001 * sin(theta5) * cos(theta4) *
              cos(theta2 + theta3) -
          0.16600000000000001 * sin(theta2 + theta3) * cos(theta5) -
          0.23000000000000001 * sin(theta2 + theta3),
      0.16350000000000001 * sin(theta4) * cos(theta2 + theta3) -
          0.16600000000000001 * sin(theta5) * cos(theta4) *
              cos(theta2 + theta3) -
          0.16600000000000001 * sin(theta2 + theta3) * cos(theta5) -
          0.23000000000000001 * sin(theta2 + theta3),
      0.16600000000000001 * sin(theta4) * sin(theta5) * sin(theta2 + theta3) +
          0.16350000000000001 * sin(theta2 + theta3) * cos(theta4),
      -0.16600000000000001 * sin(theta5) * cos(theta2 + theta3) -
          0.16600000000000001 * sin(theta2 + theta3) * cos(theta4) *
              cos(theta5),  // cppcheck-suppress constStatement
      0;

  J.row(3) << 0, -sin(theta1), -sin(theta1),
      sin(theta2) * cos(theta1) * cos(theta3) +
          sin(theta3) * cos(theta1) * cos(theta2),
      -(-sin(theta2) * sin(theta3) * cos(theta1) +
        cos(theta1) * cos(theta2) * cos(theta3)) *
              sin(theta4) -
          sin(theta1) * cos(theta4),  // cppcheck-suppress constStatement
      -(sin(theta1) * sin(theta4) -
        cos(theta1) * cos(theta4) * cos(theta2 + theta3)) *
              sin(theta5) +
          sin(theta2 + theta3) * cos(theta1) * cos(theta5);

  J.row(4) << 0, cos(theta1), cos(theta1),
      sin(theta1) * sin(theta2) * cos(theta3) +
          sin(theta1) * sin(theta3) * cos(theta2),
      -(-sin(theta1) * sin(theta2) * sin(theta3) +
        sin(theta1) * cos(theta2) * cos(theta3)) *
              sin(theta4) +
          cos(theta1) * cos(theta4),  // cppcheck-suppress constStatement
      (sin(theta1) * cos(theta4) * cos(theta2 + theta3) +
       sin(theta4) * cos(theta1)) *
              sin(theta5) +
          sin(theta1) * sin(theta2 + theta3) * cos(theta5);

  J.row(5) << 1, 0, 0, -sin(theta2) * sin(theta3) + cos(theta2) * cos(theta3),
      -(-sin(theta2) * cos(theta3) - sin(theta3) * cos(theta2)) *
          sin(theta4),  // cppcheck-suppress constStatement
      -sin(theta5) * sin(theta2 + theta3) * cos(theta4) +
          cos(theta5) * cos(theta2 + theta3);
  for (int i = 0; i < jacobianSize; i++) {
    assert(J.row(0).size() == jacobianSize);
  }
  return J;
}  // namespace a3c
}  // namespace a3c
