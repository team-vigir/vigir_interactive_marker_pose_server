cmake_minimum_required(VERSION 2.8.3)
project(vigir_interactive_marker_pose_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  visualization_msgs
  eigen_conversions
  tf_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_interactive_marker_pose_server
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp visualization_msgs eigen_conversions tf_conversions
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(interactive_marker_pose_server src/interactive_marker_pose_server.cpp)
target_link_libraries(interactive_marker_pose_server ${catkin_LIBRARIES})

install(TARGETS interactive_marker_pose_server
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


