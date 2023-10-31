# MANIPULATOR_MOTION_PLANNING

![CICD Workflow status](https://github.com/vedran97/Project808X/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/vedran97/Project808X/branch/main/graph/badge.svg)](https://codecov.io/gh/vedran97/Project808X) 
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

This repository holds the codebase for Manipulator Motion Planning project of ENPM808X at UMD,College Park.<br>
It captures Forward Kinematics (FK) and Inverse Kinematics (IK), and Task space/Joint space level motion planning, for a 6DoF Serial manipulator with 6 revolute joints. We will ensure our design produces smooth motion,gracefully handles singularities and is self collision aware. It will be a modular and configurable design, which can be used as a plug and play library for higher level decision making/path planning tasks. The library will operate on configurable, user provided DH Parameter description provided by the user.<br>
The UML folder captures the design diagrams used for this project.<br>
Group1_ENPM808X_Midterm_Proposal.pdf contains our proposal towards this project.<br>
Group1_Midterm_QuadChart.pdf contains the QuadChart for this project. <br>
Video submission can be seen in this [LINK](https://drive.google.com/file/d/1GqNPMf5ZLAHEdCDBlPzRqtGoXQJkh4Ug/view?usp=sharing) <br>
Agile Iterative Process (AIP) sprint planning notes can be seen in this [LINK](https://docs.google.com/document/d/1R59umaoEIouIo5Q6EGVjW_MC4JSzN4ydTtZ4WABHCmk/edit?usp=share_link) <br>
Agile Iterative Process (AIP) Product Backlog, Iteration Backlog, and Work Log can be seen in this [LINK](https://docs.google.com/spreadsheets/d/1G443Giy0PAcKkRJnhE9VfA6vdtWn85yaL4MikvJ04wM/edit?usp=share_link) <br>
Added IK and FK implementation.

## Authors

1. Vedant Ranade
2. Aaqib Barodawala
3. Jerry Pittman, MBA, PMP - Naval Submarine Officer and USNA Instructor

## Helpful links 

1. [Gist for adding Code Coverage](https://github.com/TommyChangUMD/cpp-boilerplate-v2#how-to-use-github-ci-to-upload-coverage-report-to-codecov)

## Install, Doxygen Documentation, cpplint, cppcheck,build,test,dependency install Instructions

```bash
# Download the code:
  git clone https://github.com/vedran97/Project808X.git
  cd Project808X
# Install dependencies:
  sudo apt install libeigen3-dev
# Configure the project and generate a native build system:
  # Must re-run this command whenever any CMakeLists.txt file has been changed.
  cmake -S ./ -B build/
# Compile and build the project:
  # rebuild only files that are modified since the last build
  cmake --build build/
  # or rebuild everything from scracth
  cmake --build build/ --clean-first
  # to see verbose output, do:
  cmake --build build/ --verbose
# Run program:
  ./build/app/shell-app
# Run tests:
  cd build/; ctest; cd -
  # or if you have newer cmake
  ctest --test-dir build/
# Build docs:
  cmake --build build/ --target docs
  # open a web browser to browse the doc
  open docs/html/index.html
# Clean
  cmake --build build/ --target clean
# Clean and start over:
  rm -rf build/
  rm -rf .cache/
  rm -rf docs/
  rm -rf Doxyfile
  rm -rf compile_commands.json
# run clang-format
  clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -vE -e "^./build/")
# run cppcheck 
  mkdir results -p && cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude --inline-suppr $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck
#run cpplint
  mkdir results -p && cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cpplint

```

## Building for code coverage

```bash
# if you don't have gcovr or lcov installed, do:
  sudo apt-get install gcovr lcov
# Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Now, do a clean compile, run unit test, and generate the covereage report
  cmake --build build/ --clean-first --target all test_coverage
# open a web browser to browse the test coverage report
  open build/test_coverage/index.html

This generates a index.html page in the build/test_coverage sub-directory that can be viewed locally in a web browser.
```

You can also get code coverage report for the *shell-app* target, instead of unit test. Repeat the previous 2 steps but with the *app_coverage* target:

``` bash
# Now, do another clean compile, run shell-app, and generate its covereage report
  cmake --build build/ --clean-first --target all app_coverage
# open a web browser to browse the test coverage report
  open build/app_coverage/index.html

This generates a index.html page in the build/app_coverage sub-directory that can be viewed locally in a web browser.
```
