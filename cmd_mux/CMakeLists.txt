cmake_minimum_required(VERSION 2.8.3)
project(cmd_mux)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      std_msgs
      geometry_msgs
      diagnostic_updater)

find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp
      std_msgs
      geometry_msgs
      diagnostic_updater)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(cmd_mux
    src/cmd_mux.cpp
    src/cmd_mux_node.cpp
    src/cmd_mux_diagnostics.cpp)
target_link_libraries(cmd_mux ${catkin_LIBRARIES})

foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest(test/system.test)
endif()
