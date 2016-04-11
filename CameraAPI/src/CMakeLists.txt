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

# Qt
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL)

#Opencv
find_package(OpenCV REQUIRED )

#Boost
find_package(Boost 1.59 REQUIRED )

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("C:/Users/berni/CameraAPI/code/CameraAPI_git/CameraAPI/src/Skeleton")
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
include_directories("C:/Program Files/glew/include/GL")
include_directories("C:/lib/glut/include/GL")


link_directories("$ENV{RSSDK_DIR}/lib/x64")
link_directories("$ENV{RSSDK_DIR}/sample/common/lib/${bitness}/${CMAKE_VS_PLATFORM_TOOLSET}")
link_directories("C:/Boost/lib/x64")
link_directories("C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64")
link_directories("C:/lib/GL/src/GL Files/lib")
link_directories("C:/Qt/Qt5.6.0/5.6/msvc2015_64/lib")

# glew
#find_package(GLEW REQUIRED)
#if (GLEW_FOUND)
#    include_directories(${GLEW_INCLUDE_DIRS})
#    link_libraries(${GLEW_LIBRARIES})
#endif()
#link_libraries("C:/Program Files/glew/lib")


# GLM
#find_package(GLM REQUIRED)
#if (GLM_FOUND)
#    include_directories(${GLM_INCLUDE_DIRS})
#    link_libraries(${GLM_LIBRARIES})
#endif()

# glfw 
#add_subdirectory(""C:/Program Files/GLFW/include/GLFW")
#include_directories("C:/Program Files/GLFW/include/GLFW")
#target_link_libraries("C:/Program Files/GLFW/lib")

# Leap
include_directories("C:/lib/leap/LeapSDK/include")
link_directories("C:/lib/leap/LeapSDK/lib/x64")


#GLUT
#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIRS})
#link_directories(${GLUT_LIBRARY_DIRS})
#add_definitions(${GLUT_DEFINITIONS})
#if(NOT GLUT_FOUND)
#    message(ERROR " GLUT not found!")
#endif(NOT GLUT_FOUND)
#link_directories("C:/lib/glut/lib")

#GL
#include_directories("C:/lib/GL/src/GL Files/include/GL")
#link_directories("C:/lib/GL/src/GL Files/lib")

#include_directories("C:/Program Files (x86)/Windows Kits/10/Include/10.0.10586.0/um/gl")
#link_directories("C:/Program Files (x86)/Windows Kits/10/Lib/10.0.10586.0/um/x64")

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
#FILE(GLOB inFiles3 RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/Skeleton/*.h")
#FILE(GLOB inFiles4 RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/Skeleton/*.cpp")
LIST(APPEND inFiles ${inFiles1} ${inFiles2} ${inFiles3} ${inFiles4})
add_executable(CameraAPI  ${inFiles})

target_link_libraries( CameraAPI ${OpenCV_LIBS} )
target_link_libraries( CameraAPI "libpxc_d")
target_link_libraries( CameraAPI "libpxcutils_d")
target_link_libraries( CameraAPI Qt5::Widgets)
target_link_libraries( CameraAPI Kinect20)
target_link_libraries( CameraAPI Qt5OpenGLd)
target_link_libraries( CameraAPI Leap)
#target_link_libraries( CameraAPI GlU32)
#target_link_libraries( CameraAPI glut32lib)

message( STATUS "variable-Test: ${CMAKE_VS_PLATFORM_TOOLSET}" )