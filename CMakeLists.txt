cmake_minimum_required(VERSION 3.8)
project(quad_sim C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Python REQUIRED)
find_package(Boost REQUIRED)

# Generate some C/C++ code using SymPy
execute_process(
  COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/eomGenerator.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add target
add_executable(solve_physics
  src/eomCoef.c
  src/eomRhs.c
  src/animStatePublisher.cpp
  src/quadEomSystem.cpp
  src/solve_physics.cpp
)
ament_target_dependencies(solve_physics rclcpp std_msgs geometry_msgs builtin_interfaces)
target_include_directories(solve_physics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(solve_physics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Move binaries to lib folder
install(TARGETS solve_physics
  DESTINATION lib/${PROJECT_NAME}
)

# Move launch files to share folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
