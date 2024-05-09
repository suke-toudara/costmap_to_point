cmake_minimum_required(VERSION 3.8)
project(costmap_to_point)

#Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)

ament_auto_find_build_dependencies()


include_directories(
 include
 #${OpenCV_INCLUDE_DIRS}        # /usr/include/opencv;/usr/include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the component target
ament_auto_add_library(costmap_to_point_component SHARED
    src/costmap_to_point_component.cpp
)

target_compile_definitions(costmap_to_point_component
    PRIVATE COSTMAP_TO_POINT_BUILDING_DLL)


rclcpp_components_register_nodes(costmap_to_point_component
    "costmap_to_point::CostmapToPointComponent")

# Create the executor target
ament_auto_add_executable(costmap_to_point_node
    src/costmap_to_point_node.cpp
)

target_link_libraries(costmap_to_point_node
    costmap_to_point_component)

ament_auto_package()




