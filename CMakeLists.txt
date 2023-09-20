cmake_minimum_required(VERSION 3.0.2)
project(geolocation)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
# add_subdirectory("docs")

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosbag
  sensor_msgs
  std_msgs
  message_generation
  tf2
  tf2_ros
)

catkin_package( )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB INNO_SDK_SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/external/lib/inno_sdk/sdk_common/*.cpp"
    "${PROJECT_SOURCE_DIR}/external/lib/inno_sdk/sdk_common/*.h")

add_library(inno_sdk ${INNO_SDK_SOURCE_FILES})
target_include_directories(inno_sdk PRIVATE "${PROJECT_SOURCE_DIR}/external/lib/inno_sdk/include}")

## Get dependencies
# Clone csv-parser
# set(CSVPARSER_DIR ${CMAKE_SOURCE_DIR}/geolocation/external/lib/csvparser)
# if(NOT EXISTS ${CSVPARSER_DIR})
#     find_package(Git REQUIRED)
#     execute_process(
#         COMMAND ${GIT_EXECUTABLE} clone https://github.com/vincentlaucsb/csv-parser.git ${CSVPARSER_DIR}
#     )
# endif()
# Add the csv-parser CMake files to the project
# add_subdirectory(${CSVPARSER_DIR})
# include_directories(${CSVPARSER_DIR}/include)

## Dummy Publishers
add_executable(dummy_pt_pub src/dummy_publishers/dummy_pt_pub.cpp)
target_link_libraries(dummy_pt_pub ${catkin_LIBRARIES})
add_dependencies(dummy_pt_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dummy_bearing_pub src/dummy_publishers/dummy_bearing_pub.cpp)
target_link_libraries(dummy_bearing_pub ${catkin_LIBRARIES})
add_dependencies(dummy_bearing_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dummy_elv_pub src/dummy_publishers/dummy_elv_pub.cpp)
target_link_libraries(dummy_elv_pub ${catkin_LIBRARIES})
add_dependencies(dummy_elv_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dummy_gnss_pub src/dummy_publishers/dummy_gnss_pub.cpp)
target_link_libraries(dummy_gnss_pub ${catkin_LIBRARIES})
add_dependencies(dummy_gnss_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dummy_markers_pub src/dummy_publishers/dummy_markers_pub.cpp)
target_link_libraries(dummy_markers_pub ${catkin_LIBRARIES})
add_dependencies(dummy_markers_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## TF2 Broadcasters

add_executable(map_tf2_broadcaster src/tf2_broadcasts/map_tf2_broadcaster.cpp)
target_link_libraries(map_tf2_broadcaster ${catkin_LIBRARIES})
add_dependencies(map_tf2_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(base_tf2_broadcaster src/tf2_broadcasts/base_tf2_broadcaster.cpp)
target_link_libraries(base_tf2_broadcaster ${catkin_LIBRARIES})
add_dependencies(base_tf2_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Utilities

add_executable(bag_gps_merge src/utilities/bag_gps_merge.cpp)
target_link_libraries(bag_gps_merge ${catkin_LIBRARIES})
add_dependencies(bag_gps_merge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(bag_csv_merge src/utilities/bag_csv_merge.cpp)
# target_link_libraries(bag_csv_merge ${catkin_LIBRARIES} csv)
# add_dependencies(bag_csv_merge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(test_roi_move src/utilities/test_roi_move.cpp)
# target_link_libraries(test_roi_move ${catkin_LIBRARIES} inno_sdk)
# add_dependencies(test_roi_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/geo_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
