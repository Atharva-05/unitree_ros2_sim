cmake_minimum_required(VERSION 3.8)
project(unitree_guide2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROBOT_TYPE Go1)         # The type of robot, support Go1 and A1 currently
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64

set(CATKIN_MAKE ON)        # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION ON)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF
set(MOVE_BASE ON)               # Need move_base or not, ON or OFF

if((CATKIN_MAKE) AND (SIMULATION) AND (MOVE_BASE))
    set(ROS_2 ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(SIMULATION OR MOVE_BASE OR ROS_2)
    add_definitions(-DRUN_ROS)
    set(CATKIN_MAKE ON)
endif()

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

set(CMAKE_CXX_STANDARD 11)
find_package(Boost REQUIRED)

# if(CATKIN_MAKE)
#     add_definitions(-DCOMPILE_WITH_ROS)
#     if(MOVE_BASE)
#         add_definitions(-DCOMPILE_WITH_MOVE_BASE)
#     endif()
# endif()

if(ROS_2)
    add_definitions(-DCOMPILE_WITH_ROS2_MB)
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()

if(CATKIN_MAKE)
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(controller_manager REQUIRED)
        find_package(joint_trajectory_controller REQUIRED)
        find_package(gazebo_ros REQUIRED)
    endif()

    # find_package(rosidl_generator REQUIRED)
    find_package(robot_state_publisher REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(ros2_unitree_legged_msgs REQUIRED)
endif()

include_directories(
    include
    ${Boost_INCLUDEDIR}
    ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

if(REAL_ROBOT)
  add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
  if(${ROBOT_TYPE} STREQUAL "A1")
      include_directories(
          library/unitree_legged_sdk_3.2/include
      )
      link_directories(
          library/unitree_legged_sdk_3.2/lib
      )
  elseif(${ROBOT_TYPE} STREQUAL "Go1")
      include_directories(
          library/unitree_legged_sdk-3.8.0/include
      )
      if(${PLATFORM} STREQUAL "amd64")
          link_directories(
              library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
          )
      elseif(${PLATFORM} STREQUAL "arm64")
          link_directories(
              library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
          )
      endif()
  endif()
endif()

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})

if(CATKIN_MAKE)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
    # add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    ament_target_dependencies(junior_ctrl rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs geometry_msgs ros2_unitree_legged_msgs Boost)

endif()
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
endif()
if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
            target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    endif()
endif()
target_link_libraries(junior_ctrl -pthread lcm)

if(NOT CATKIN_MAKE)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
endif()

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

install(TARGETS junior_ctrl DESTINATION lib/${PROJECT_NAME})

ament_package()
