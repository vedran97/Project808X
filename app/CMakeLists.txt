# find_package(Eigen3 REQUIRED)

# Any C++ source files needed to build this target (shell-app).
add_executable(shell-app
  # list of source cpp files:
  main.cpp

  )

# link_directories(${Eigen_INCLUDE_DIRS})
# Any include directories needed to build this target.
# Note: we do not need to specify the include directories for the
# dependent libraries, they are automatically included.
target_include_directories(shell-app PUBLIC
  # list inclue directories:
  ${CMAKE_SOURCE_DIR}/include
  
)

# Any dependent libraires needed to build this target.
target_link_libraries(shell-app PUBLIC
  # list of libraries

  )

target_link_options(shell-app PUBLIC
  --static
  )
  