#include <gtest/gtest.h>

#include "include/ForwardKinematics.hpp"
#include "include/InverseKinematics.hpp"

TEST(FK_Test, test_EF_position_1) {
  auto fk = a3c::ForwardKinematics();
  a3c::JointAngles ja = {{0, 0, 0, 0, 0, 0}};
  auto pose = fk.fk(ja);
  Eigen::Vector3d expectedPosition = {0, 0.163, 0.8175};
  EXPECT_EQ(pose.position, expectedPosition);
}

TEST(FK_Test, test_EF_position_2) {
  auto fk = a3c::ForwardKinematics();
  a3c::JointAngles ja = {{0, 0, -90, 0, 0, 0}};
  auto pose = fk.fk(ja);
  Eigen::Vector3d expectedPosition = {0.396, 0.163, 0.4215};
  EXPECT_EQ(pose.position, expectedPosition);
}

TEST(IK_Test, test_JA_position_1) {
  Eigen::Matrix4d transormationMatrix;
  const a3c::JointAngles currentAngles = {{0, 0, 0, 0, 0, 0}};
  const a3c::JointAngles targetAngles = {{0, 0, 0, 0, 0, 0}};
  auto ik = a3c::InverseKinematics(transormationMatrix);
  auto trajectory = ik.ik(currentAngles, targetAngles);
  a3c::JointAngles expectedJA = {{0, 0, -90, 0, 0, 0}};
  std::vector<a3c::JointAngles> expectedTraj = {expectedJA};
  EXPECT_EQ(trajectory, expectedTraj);
}
