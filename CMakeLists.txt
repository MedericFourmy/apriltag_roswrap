cmake_minimum_required(VERSION 2.8.3)
project(apriltag_roswrap)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


# ******************************************************************** 
#                 Add catkin additional components here
#                 Find catkin macros and libraries
# ******************************************************************** 
# Ros related dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  geometry_msgs
  image_transport
  std_msgs
  tf
  cv_bridge
  tf_conversions
  OpenCV
)

# Other dependencies
find_package(eigen_conversions REQUIRED)

FIND_PATH(APRILTAG_INCLUDE_DIR NAMES apriltag.h  PATH_SUFFIXES "apriltag" ${APRILTAG_INCLUDE_PATH})
FIND_LIBRARY(APRILTAG_LIBRARY NAMES apriltag PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}" "apriltag" ${APRILTAG_LIBRARY_PATH})
IF(APRILTAG_LIBRARY)
    SET(Apriltag_FOUND TRUE)
    MESSAGE("apriltag Library FOUND in ${APRILTAG_LIBRARY}: apriltag related sources will be built.")
ENDIF(APRILTAG_LIBRARY)



# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate  messages in the 'msg' and 'srv' folders
add_message_files(
  FILES
  Tag.msg
  TagArray.msg
)
# add_service_files(
#   FILES
#   SaveTraj.srv
# )

# ******************************************************************** 
#           Generate messages and services with dependencies below
# ******************************************************************** 
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
#generate_dynamic_reconfigure_options(cfg/Wolf.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs std_srvs
# ******************************************************************** 
#      Add run time dependencies here
# ******************************************************************** 
#   DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    # ${EIGEN_INCLUDE_DIRS}
                    ${APRILTAG_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(tag_detection src/tag_detection.cpp)
add_dependencies(tag_detection ${PROJECT_NAME}_generate_messages_cpp )

## Declare a C++ executable
add_executable(tag_detection_node src/tag_detection_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(tag_detection ${catkin_LIBRARIES})
target_link_libraries(tag_detection ${OpenCV_LIBS})
target_link_libraries(tag_detection ${APRILTAG_LIBRARY})
target_link_libraries(tag_detection_node tag_detection)
