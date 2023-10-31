#include <gtest/gtest.h>

#include "include/ForwardKinematics.hpp"
#include "include/InverseKinematics.hpp"
/**
@brief Test the Forward Kinematics Class for All angles at 0 radians
*/
TEST(FK_Test, test_EF_position_1) {
  auto fk = a3c::ForwardKinematics();
  a3c::JointAngles ja = {{0, 0, 0, 0, 0, 0}};
  auto pose = fk.fk(ja);
  Eigen::Vector3d expectedPosition = {0, 0.163, 0.8175};
  EXPECT_NEAR(pose.position.x(), expectedPosition.x(), 1E2);
  EXPECT_NEAR(pose.position.y(), expectedPosition.y(), 1E2);
  EXPECT_NEAR(pose.position.z(), expectedPosition.z(), 1E2);
}

/**
  @brief Test the Forward Kinematics Class for All angles expect J3 at 0.
  J3=-1.57 radians
*/
TEST(FK_Test, test_EF_position_2) {
  auto fk = a3c::ForwardKinematics();
  a3c::JointAngles ja = {{0, 0, -1.57, 0, 0, 0}};
  auto pose = fk.fk(ja);
  Eigen::Vector3d expectedPosition = {0.396, 0.163, 0.4215};
  EXPECT_NEAR(pose.position.x(), expectedPosition.x(), 1E2);
  EXPECT_NEAR(pose.position.y(), expectedPosition.y(), 1E2);
  EXPECT_NEAR(pose.position.z(), expectedPosition.z(), 1E2);
}

/**
  @brief Test the IK functionality
  @note verifies value of jacobian at angles = {0,0,0,0,0,0}
*/
TEST(IK_Test, test_jacobian) {
  const a3c::JointAngles currentAngles = {{0, 0, 0, 0, 0, 0}};
  auto ik = a3c::InverseKinematics(currentAngles);
  auto jac = ik.getJacobian(currentAngles);
  MatrixXd expectedJac(6, 6);
  expectedJac = expectedJac.Zero(6, 6);
  expectedJac.row(0) << -0.163, 0.626, 0.396, -0.1653,
      0.166,  // cppcheck-suppress constStatement
      0;
  expectedJac.row(4) << 0, 1, 1, 0, 1, 0;  // cppcheck-suppress constStatement
  expectedJac.row(5) << 1, 0, 0, 1, 0, 1;  // cppcheck-suppress constStatement
  for (int i = 0; i < expectedJac.rows(); i++) {
    for (int j = 0; j < expectedJac.cols(); j++) {
      EXPECT_NEAR(jac.coeff(i, j), expectedJac.coeff(i, j), 1E4);
    }
  }
}
