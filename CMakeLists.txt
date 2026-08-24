cmake_minimum_required(VERSION 3.0.2)
project(daib_explorer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  daib_decision
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(PCL REQUIRED COMPONENTS common filters)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES daib_explorer_core
  CATKIN_DEPENDS daib_decision geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(daib_explorer_core src/explorer_core.cpp src/pvbsm_memory.cpp)
target_link_libraries(daib_explorer_core ${catkin_LIBRARIES})

add_executable(daib_explorer_node src/explorer_node.cpp)
target_link_libraries(daib_explorer_node daib_explorer_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(explorer_core_test test/explorer_core_test.cpp)
  if(TARGET explorer_core_test)
    target_link_libraries(explorer_core_test daib_explorer_core)
  endif()
  add_rostest(test/runtime_contract.test)
endif()

install(TARGETS daib_explorer_core daib_explorer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
