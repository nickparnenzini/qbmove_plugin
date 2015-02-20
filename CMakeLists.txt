# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_executable (talker talker.cc)
target_link_libraries(talker position_stiffness_creator_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(talker position_stiffness_creator_msgs)
target_link_libraries(talker pos_current_echo_creator_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(talker pos_current_echo_creator_msgs)

add_library(qbmove_plugin SHARED qbmove_plugin.cc )
target_link_libraries(qbmove_plugin position_stiffness_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(qbmove_plugin position_stiffness_creator_msgs)
target_link_libraries(qbmove_plugin pos_current_echo_creator_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(qbmove_plugin pos_current_echo_creator_msgs)
