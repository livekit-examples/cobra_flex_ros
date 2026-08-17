#
# Copyright 2026 LiveKit
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(cobra_flex_pan_tilt)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(fmt REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Vendored Feetech SCServo SDK (github.com/adityakamath/SCServo_Linux).
# Internal static lib only; nothing is installed or exported.
add_library(scservo STATIC
  scservo/src/SCS.cpp
  scservo/src/SCSerial.cpp
  scservo/src/SMS_STS.cpp
  scservo/src/SCSCL.cpp
  scservo/src/HLSCL.cpp)
target_include_directories(scservo
  # Consumers include "scservo/SCServo.h"; the SDK's own sources use flat
  # includes ("SCS.h"), hence the second, private include dir.
  PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/scservo/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/scservo/include/scservo)
set_target_properties(scservo PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_auto_add_executable(pan_tilt_driver
  src/pan_tilt_node.cpp
  src/pan_tilt_controller.cpp)
# Header-only fmt: rclcpp already pulls in spdlog's libfmt.so.8, and linking
# a second, newer libfmt shared object alongside it risks symbol clashes.
target_link_libraries(pan_tilt_driver scservo fmt::fmt-header-only)

ament_auto_add_executable(pan_tilt_keyboard_teleop
  src/pan_tilt_keyboard_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # pan_tilt_controller.* was derived from the teleop pan_tilt_demo (and has
  # since diverged), and the vendored SCServo SDK is a verbatim upstream copy
  # that must stay diffable; skip the C++ style linters over both rather than
  # reformatting them, and run the copyright check only on the files this
  # package authors (ament_copyright also cannot parse the upstream
  # block-comment license headers).
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify ament_cmake_cpplint ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright("src/pan_tilt_node.cpp" "src/pan_tilt_keyboard_node.cpp"
    "launch" "package.xml" "CMakeLists.txt")
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE launch config)
