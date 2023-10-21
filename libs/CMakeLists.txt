find_package(Eigen3 REQUIRED)

link_directories(${Eigen_INCLUDE_DIRS})

add_library(Kinematics
    IK.cpp
    FK.cpp
)

target_include_directories(Kinematics PUBLIC

    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)
