cmake_minimum_required(VERSION 3.16)
project(ros2_unitree_legged_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_unitree_legged_msgs
  control_toolbox
  urdf
  urdf_parser_plugin
)

# set(BUILD_SHARED_LIBS ON)
# link_directories(lib)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(urdf REQUIRED)

# Creates parameter struct
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  ros2_unitree_legged_control_parameters
  src/ros2_unitree_legged_control.yaml
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(TOOL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include/ros2_unitree_legged_control")
set(TOOL_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_library(unitree_joint_control_tool STATIC IMPORTED SHARED)
set_target_properties(unitree_joint_control_tool PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/libunitree_joint_control_tool.so)

add_library(ros2_unitree_legged_control SHARED
  src/ros2_unitree_legged_control.cpp
)
# include_directories(${TOOL_INCLUDE})
# link_directories(${TOOL_LIB})
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
# include_directories(include/)
# link_directories(lib/)
# target_link_directories(unitree_joint_control_tool lib/)

# add_library(unitree_joint_control_tool SHARED lib/libunitree_joint_control_tool.so)
# set_target_properties(unitree_joint_control_tool PROPERTIES LINKER_LANGUAGE CXX)
# ament_export_targets(unitree_joint_control_tool HAS_LIBRARY_TARGET)

target_link_libraries(ros2_unitree_legged_control PUBLIC
  ros2_unitree_legged_control_parameters
  unitree_joint_control_tool
)
target_compile_features(ros2_unitree_legged_control PUBLIC cxx_std_17)
target_include_directories(ros2_unitree_legged_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ros2_unitree_legged_control>
)


ament_target_dependencies(ros2_unitree_legged_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS} ros2_unitree_legged_msgs control_toolbox urdf)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_unitree_legged_control PRIVATE "ROS2_UNITREE_CONTROL_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface ros2_unitree_legged_control.xml)

# TODO
# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   ament_add_gmock(test_load_joint_group_effort_controller
#     test/test_load_joint_group_effort_controller.cpp
#   )
#   target_link_libraries(test_load_joint_group_effort_controller
#     effort_controllers
#   )
#   ament_target_dependencies(test_load_joint_group_effort_controller
#     controller_manager
#     ros2_control_test_assets
#   )

#   ament_add_gmock(test_joint_group_effort_controller
#     test/test_joint_group_effort_controller.cpp
#   )
#   target_link_libraries(test_joint_group_effort_controller
#     effort_controllers
#   )
# endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
# install(FILES
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/libunitree_joint_control_tool.so
#   DESTINATION lib
# )
install(TARGETS
    ros2_unitree_legged_control
    ros2_unitree_legged_control_parameters
    # unitree_joint_control_tool
  EXPORT ros2_unitree_legged_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${CMAKE_SOURCE_DIR}/lib)

ament_export_targets(ros2_unitree_legged_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
