/**
 * @file IK.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief Inverse Kinematics class implementation
 * @version 0.1
 * @date 2023-10-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <cmath>
#include <iostream>

#include "Eigen/src/Core/Matrix.h"
#include "include/InverseKinematics.hpp"

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
std::vector<JointAngles> InverseKinematics::ik(
    const JointAngles& currentAngles, const JointAngles& targetAngles) {
  
  const int size = 6;
  MatrixXd J(size, size);
  J.row(0) = -0.23000000000000001*sin(theta1)*sin(theta2) + 0.16350000000000001*sin(theta1)*sin(theta4)*cos(theta2 + theta3) - 0.16600000000000001*sin(theta1)*sin(theta5)*cos(theta4)*cos(theta2 + theta3) - 0.16600000000000001*sin(theta1)*sin(theta2 + theta3)*cos(theta5) - 0.23000000000000001*sin(theta1)*sin(theta2 + theta3) - 0.16600000000000001*sin(theta4)*sin(theta5)*cos(theta1) - 0.16350000000000001*cos(theta1)*cos(theta4) + 0.00050000000000000001*cos(theta1), 
    0.16350000000000001*sin(theta4)*sin(theta2 + theta3)*cos(theta1) - 0.16600000000000001*sin(theta5)*sin(theta2 + theta3)*cos(theta1)*cos(theta4) + 0.23000000000000001*cos(theta1)*cos(theta2) + 0.16600000000000001*cos(theta1)*cos(theta5)*cos(theta2 + theta3) + 0.23000000000000001*cos(theta1)*cos(theta2 + theta3), 
    0.16350000000000001*sin(theta4)*sin(theta2 + theta3)*cos(theta1) - 0.16600000000000001*sin(theta5)*sin(theta2 + theta3)*cos(theta1)*cos(theta4) + 0.16600000000000001*cos(theta1)*cos(theta5)*cos(theta2 + theta3) + 0.23000000000000001*cos(theta1)*cos(theta2 + theta3), 
    0.16350000000000001*sin(theta1)*sin(theta4) - 0.16600000000000001*sin(theta1)*sin(theta5)*cos(theta4) - 0.16600000000000001*sin(theta4)*sin(theta5)*cos(theta1)*cos(theta2 + theta3) - 0.16350000000000001*cos(theta1)*cos(theta4)*cos(theta2 + theta3), 
    -0.16600000000000001*sin(theta1)*sin(theta4)*cos(theta5) - 0.16600000000000001*sin(theta5)*sin(theta2 + theta3)*cos(theta1) + 0.16600000000000001*cos(theta1)*cos(theta4)*cos(theta5)*cos(theta2 + theta3), 
    0;

  J.row(1) = -0.16600000000000001*sin(theta1)*sin(theta4)*sin(theta5) - 0.16350000000000001*sin(theta1)*cos(theta4) + 0.00050000000000000001*sin(theta1) + 0.23000000000000001*sin(theta2)*cos(theta1) - 0.16350000000000001*sin(theta4)*cos(theta1)*cos(theta2 + theta3) + 0.16600000000000001*sin(theta5)*cos(theta1)*cos(theta4)*cos(theta2 + theta3) + 0.16600000000000001*sin(theta2 + theta3)*cos(theta1)*cos(theta5) + 0.23000000000000001*sin(theta2 + theta3)*cos(theta1),
    0.16350000000000001*sin(theta1)*sin(theta4)*sin(theta2 + theta3) - 0.16600000000000001*sin(theta1)*sin(theta5)*sin(theta2 + theta3)*cos(theta4) + 0.23000000000000001*sin(theta1)*cos(theta2) + 0.16600000000000001*sin(theta1)*cos(theta5)*cos(theta2 + theta3) + 0.23000000000000001*sin(theta1)*cos(theta2 + theta3),
    0.16350000000000001*sin(theta1)*sin(theta4)*sin(theta2 + theta3) - 0.16600000000000001*sin(theta1)*sin(theta5)*sin(theta2 + theta3)*cos(theta4) + 0.16600000000000001*sin(theta1)*cos(theta5)*cos(theta2 + theta3) + 0.23000000000000001*sin(theta1)*cos(theta2 + theta3),
    -0.16600000000000001*sin(theta1)*sin(theta4)*sin(theta5)*cos(theta2 + theta3) - 0.16350000000000001*sin(theta1)*cos(theta4)*cos(theta2 + theta3) - 0.16350000000000001*sin(theta4)*cos(theta1) + 0.16600000000000001*sin(theta5)*cos(theta1)*cos(theta4),
    -0.16600000000000001*sin(theta1)*sin(theta5)*sin(theta2 + theta3) + 0.16600000000000001*sin(theta1)*cos(theta4)*cos(theta5)*cos(theta2 + theta3) + 0.16600000000000001*sin(theta4)*cos(theta1)*cos(theta5);
    0;

  J.row(2) = 0,
     -0.23000000000000001*sin(theta2) + 0.16350000000000001*sin(theta4)*cos(theta2 + theta3) - 0.16600000000000001*sin(theta5)*cos(theta4)*cos(theta2 + theta3) - 0.16600000000000001*sin(theta2 + theta3)*cos(theta5) - 0.23000000000000001*sin(theta2 + theta3),
     0.16350000000000001*sin(theta4)*cos(theta2 + theta3) - 0.16600000000000001*sin(theta5)*cos(theta4)*cos(theta2 + theta3) - 0.16600000000000001*sin(theta2 + theta3)*cos(theta5) - 0.23000000000000001*sin(theta2 + theta3),
     0.16600000000000001*sin(theta4)*sin(theta5)*sin(theta2 + theta3) + 0.16350000000000001*sin(theta2 + theta3)*cos(theta4),
     -0.16600000000000001*sin(theta5)*cos(theta2 + theta3) - 0.16600000000000001*sin(theta2 + theta3)*cos(theta4)*cos(theta5),
     0;


  J.row(3) = 0,
    -sin(theta1),
    -sin(theta1),
    sin(theta2)*cos(theta1)*cos(theta3) + sin(theta3)*cos(theta1)*cos(theta2),
    -(-sin(theta2)*sin(theta3)*cos(theta1) + cos(theta1)*cos(theta2)*cos(theta3))*sin(theta4) - sin(theta1)*cos(theta4),
    -(sin(theta1)*sin(theta4) - cos(theta1)*cos(theta4)*cos(theta2 + theta3))*sin(theta5) + sin(theta2 + theta3)*cos(theta1)*cos(theta5);

  J.row(4) = 0,
    cos(theta1),
    cos(theta1),
    sin(theta1)*sin(theta2)*cos(theta3) + sin(theta1)*sin(theta3)*cos(theta2),
    -(-sin(theta1)*sin(theta2)*sin(theta3) + sin(theta1)*cos(theta2)*cos(theta3))*sin(theta4) + cos(theta1)*cos(theta4),
    (sin(theta1)*cos(theta4)*cos(theta2 + theta3) + sin(theta4)*cos(theta1))*sin(theta5) + sin(theta1)*sin(theta2 + theta3)*cos(theta5);
  
  J.row(5) = 1,
    0,
    0,
    -sin(theta2)*sin(theta3) + cos(theta2)*cos(theta3),
    -(-sin(theta2)*cos(theta3) - sin(theta3)*cos(theta2))*sin(theta4),
    -sin(theta5)*sin(theta2 + theta3)*cos(theta4) + cos(theta5)*cos(theta2 + theta3);

  // TODO(aaqibsb): Stuff.
  return {};
}
}  // namespace a3c
