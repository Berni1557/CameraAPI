cmake_minimum_required (VERSION 2.6)
project (CameraAPI C CXX)
message(STATUS "Starting CameraAPI generation!")

#set_target_properties(CameraAPI PROPERTIES LINKER_LANGUAGE CPP)

# get Architecture and set variable bitness
message(STATUS "Starting CameraAPI generation!")
set(bitness win32)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(bitness x64)
endif()

# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)
 
# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/main.h"
  "${PROJECT_BINARY_DIR}/main.h"
  )
# configure_file (
  # "${PROJECT_SOURCE_DIR}/CameraAPI.h"
  # "${PROJECT_BINARY_DIR}/CameraAPI.h"
# )
# configure_file (
  # "${PROJECT_SOURCE_DIR}/IntelAPI.h"
  # "${PROJECT_BINARY_DIR}/IntelAPI.h"
# )
# configure_file (
  # "${PROJECT_SOURCE_DIR}/CameraRGB.h"
  # "${PROJECT_BINARY_DIR}/CameraRGB.h"
# )

find_package(Qt5Widgets REQUIRED)
find_package(OpenCV REQUIRED )
find_package(Boost 1.59 REQUIRED )

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("$ENV{RSSDK_DIR}/include")
include_directories("$ENV{RSSDK_DIR}/sample/common/include")
#include_directories("$ENV{EIGEN3_INCLUDE_DIR}" )
#include_directories(EIGEN3_INCLUDE_DIR)
include_directories("C:/boost/include/boost-1_59")
include_directories("C:/Program Files/Eigen/include/eigen3")
include_directories("C:/Program Files/Eigen/include/eigen3")
#include_directories("C:/Qt/Qt5.6.0/5.6/msvc2015_64/include/QtGui")
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories("C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/inc")

link_directories("$ENV{RSSDK_DIR}/lib/x64")
link_directories("$ENV{RSSDK_DIR}/sample/common/lib/${bitness}/${CMAKE_VS_PLATFORM_TOOLSET}")
link_directories("C:/Boost/lib/x64")
link_directories("C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64")

# Find Kinect SDK v2
#find_package( KinectSDK2 REQUIRED )

# Settings Kinect SDK v2
#if(KinectSDK2_FOUND)
#  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
#  link_directories( ${KinectSDK2_LIBRARY_DIRS} )
#  target_link_libraries( project ${KinectSDK2_LIBRARIES} )
#  add_custom_command( TARGET project POST_BUILD ${KinectSDK2_COMMANDS} )
#endif()


#find_package(Boost)
# add the executable

FILE(GLOB inFiles1 RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
FILE(GLOB inFiles2 RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
LIST(APPEND inFiles ${inFiles1} ${inFiles2})
add_executable(CameraAPI  ${inFiles})

target_link_libraries( CameraAPI ${OpenCV_LIBS} )
target_link_libraries( CameraAPI "libpxc_d")
target_link_libraries( CameraAPI "libpxcutils_d")
target_link_libraries( CameraAPI Qt5::Widgets)
target_link_libraries( CameraAPI Kinect20)


message( STATUS "variable-Test: ${CMAKE_VS_PLATFORM_TOOLSET}" )