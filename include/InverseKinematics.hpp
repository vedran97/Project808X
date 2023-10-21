/**
 * @file InverseKinematics.hpp
 * @author Jerry Pittman, Jr. (jpittma1@umd.edu)
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
#include <iostream>
#include <iterator>
#include <string>
#include <eigen3/Eigen/Dense>
#include <numeric>
#include <vector>

#include "ForwardKinematics.hpp"

using namespace Eigen;

namespace a3c
{
    using JointAngles = std::array<double, 6>;
    
    /**
     * @brief Class IK
     * 
     */
    class InverseKinematics
    {
        
    private:
        using DHTable = Eigen::Array<double, ForwardKinematics::mNumDHRows, ForwardKinematics::mNumDHCols>;
        Matrix4d ForwardTransformationMatrix;

    public:
        InverseKinematics() noexcept;
        std::vector <JointAngles> ik(const JointAngles& currentAngles, const JointAngles& targetAngles);
    };
}

#endif