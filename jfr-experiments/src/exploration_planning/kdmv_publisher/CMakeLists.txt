
# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(kdmv_publisher)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.



find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp pcl_ros
  std_msgs
  nav_msgs
  tf
  cv_bridge 
  sensor_msgs 
  visualization_msgs
  image_transport
  geometry_msgs
)


generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
    DEPENDS eigen opencv2 libpcl-all-dev
    CATKIN_DEPENDS cv_bridge roscpp pcl_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

#find_package(OpenCV REQUIRED)



#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf geometry_msgs image_transport cv_bridge sensor_msgs visualization_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")
add_executable(kdmv_publisher src/kernel_dmv_node.cpp src/gas_map.cpp)


find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

target_link_libraries(kdmv_publisher ${catkin_LIBRARIES} )

#uncomment if you have defined messages


#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(tf)



#---- GDM Publisher
#=======================================
add_executable(gdm_publisher_node src/gdm_publisher_node.cpp src/gas_map.cpp)
target_link_libraries(gdm_publisher_node ${catkin_LIBRARIES} )

#---- Simple test Publisher
#=======================================
add_executable(simple_test_publisher_node src/simple_test_publisher_node.cpp src/gas_map.cpp)
target_link_libraries(simple_test_publisher_node ${catkin_LIBRARIES} )


