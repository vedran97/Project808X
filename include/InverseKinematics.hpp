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
        constexpr static const size_t mNumDHRows = 6;
        constexpr static const size_t mNumDHCols = 4;
        constexpr static const size_t alphaIndex = 0;
        constexpr static const size_t aIndex = 1;
        constexpr static const size_t dIndex = 2;
        constexpr static const size_t thetaIndex = 3;
        using DHTable = Eigen::Array<double, mNumDHRows, mNumDHCols>;
        DHTable dhTable;
        /**
         * Declaring these for the A3C here, later can be read from a file in the constructor and set
         */
        const DHParams dhParams{0.1915, 0.1405, 0.1415, 0.230, 0.1635, 0.1665, 0.230};

    public:
        InverseKinematics() noexcept;
        Matrix4d getTransformationMatrix(const Eigen::Array<double, 1, mNumDHCols> &dhRow) const noexcept;
    };
}

#endif