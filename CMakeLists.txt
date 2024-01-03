cmake_minimum_required(VERSION 3.8)

# Create project and make it's name available to the executables.
# This will allow the nodes to figure out the name of the package they are in.
project(quad_sim C CXX)
add_compile_definitions(ROS_PACKAGE_NAME="${CMAKE_PROJECT_NAME}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(quad_sim_interfaces REQUIRED)
find_package(Python REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate some C/C++ code using SymPy
execute_process(
  COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/eomGenerator.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add target
add_executable(solve_physics
  src/eomCoef.c
  src/eomRhs.c
  src/propVelocities.c
  src/propThTq.cpp
  src/motorTq.cpp
  src/simTimePublisher.cpp
  src/animStatePublisher.cpp
  src/sensorDataPublisher.cpp
  src/controllerInSubscriber.cpp
  src/quadEomSystem.cpp
  src/solve_physics.cpp
)
ament_target_dependencies(solve_physics rclcpp std_msgs geometry_msgs builtin_interfaces rosgraph_msgs quad_sim_interfaces sensor_msgs)
target_link_libraries(solve_physics Eigen3::Eigen)
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
  propData
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
