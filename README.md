# MANIPULATOR_MOTION_PLANNING

## Overview

This repository holds the codebase for Manipulator Motion Planning project of ENPM808X at UMD,College Park.<br>
It captures FK and self collision avoidance aware IK, and Task space/Joint space level motion planning, for a 6DoF Serial manipulator.<br>
More details to come soon!<br>

## Authors

1. Vedant Ranade
2. Aaqib Barodawala
3. Jerry Pittman

## Helpful links 

1. [Gist for adding Code Coverage](https://github.com/TommyChangUMD/cpp-boilerplate-v2#how-to-use-github-ci-to-upload-coverage-report-to-codecov)

## Install instructions

```bash
# Download the code:
  git clone https://github.com/vedran97/Project808X.git
  cd Project808X
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
  mkdir output -p && cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cppcheck
#run cpplint
  mkdir output -p && cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" ) &> results/cpplint

```