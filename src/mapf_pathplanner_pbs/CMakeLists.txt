cmake_minimum_required(VERSION 3.5)
project(mapf_pathplanner_pbs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mapf_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)

include_directories(include/mapf_pathplanner_pbs)

add_executable(pathplanner src/pathplanner.cpp src/Instance.cpp src/PBS.cpp src/PBSNode.cpp 
              src/ReservationTable.cpp src/SingleAgentSolver.cpp src/SIPP.cpp src/SpaceTimeAStar.cpp
              src/common.cpp src/Conflict.cpp src/ConstraintTable.cpp)
ament_target_dependencies(pathplanner rclcpp mapf_interfaces)

install(TARGETS
  pathplanner
  DESTINATION lib/${PROJECT_NAME})

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
