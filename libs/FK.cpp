/**
 * @file FK.cpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
 * @brief FK definitions and implementation
 * @version 0.1
 * @date 2023-10-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <cmath>
#include <iostream>

#include "include/ForwardKinematics.hpp"

namespace a3c {
/**
 * @brief Construct a new Forward Kinematics:: Forward Kinematics object
 *
 */
ForwardKinematics::ForwardKinematics() noexcept {
  /*
  initialize DH table in this constructor
  */
  (dhTable << 0, 0, dhParams.d1, 0, -M_PI_2, 0, dhParams.d2, -M_PI_2, 0,
   dhParams.a2, -dhParams.d3, M_PI_2, M_PI_2, 0, dhParams.d4, 0, -M_PI_2, 0,
   dhParams.d5, 0, M_PI_2, 0, dhParams.d6, M_PI_2)
      .finished();
}
/**
 * @brief Create fk Pose method
 * @param jointAngles Joint angles in radians
 * @return Pose: Pose of the End Effector
 */
Pose ForwardKinematics::fk(const JointAngles &jointAngles) noexcept {
  Matrix4d T = Matrix4d::Identity();
  for (size_t i = 0; i < mNumDHRows; ++i) {
    dhTable(i, thetaIndex) += jointAngles.at(i);
    T *= getTransformationMatrix(dhTable.row(i));
    dhTable(i, thetaIndex) -= jointAngles.at(i);
  }
  return Pose(T);
}
/**
 * @brief Gets Transformation Matrix
 *
 * @param dhRow
 * @return Matrix4d
 */
Matrix4d ForwardKinematics::getTransformationMatrix(
    const Eigen::Array<double, 1, mNumDHCols> &dhRow) const noexcept {
  double sinTheta = sin(dhRow(thetaIndex));
  double cosTheta = cos(dhRow(thetaIndex));
  double sinAlpha = sin(dhRow(alphaIndex));
  double cosAlpha = cos(dhRow(alphaIndex));
  Matrix4d T;
  (T << cosTheta, -sinTheta, 0, dhRow(aIndex), sinTheta * cosAlpha,
   cosTheta * cosAlpha, -sinAlpha, -sinAlpha * dhRow(dIndex),
   sinTheta * sinAlpha, cosTheta * sinAlpha, cosAlpha, cosAlpha * dhRow(dIndex),
   0, 0, 0, 1); // cppcheck-suppress constStatement
  return T;
}
/**
 * @brief Construct a new DHParams::DHParams object
 *
 * @param d1
 * @param d2
 * @param d3
 * @param d4
 * @param d5
 * @param d6
 * @param a2
 */
DHParams::DHParams(float d1, float d2, float d3, float d4, float d5, float d6,
                   float a2)
    : d1(d1), d2(d2), d3(d3), d4(d4), d5(d5), d6(d6), a2(a2) {}

/**
 * @brief Override ostream
 *
 * @param os
 * @param pose
 * @return std::ostream&
 */
std::ostream &operator<<(std::ostream &os, const Pose &pose) {
  os << "Position: \r\n"
     << "x:" << pose.position.x() << "\r\n"
     << "y:" << pose.position.y() << "\r\n"
     << "z:" << pose.position.z() << "\r\n"
     << "Orientation: \r\n"
     << "x:" << pose.orientation.x() << "\r\n"
     << "y:" << pose.orientation.y() << "\r\n"
     << "z:" << pose.orientation.z() << "\r\n"
     << "w:" << pose.orientation.w() << "\r\n\r\n";
  return os;
}
}  // namespace a3c
