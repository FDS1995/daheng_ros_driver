cmake_minimum_required(VERSION 2.8.3)
project(daheng_ros)
add_definitions(
    -std=c++11
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)


find_package(OpenCV)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /home/fds/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/genicam/library/CPP/include
  /home/fds/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/include
)
link_directories(
    /home/fds/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/genicam/bin/Linux64_x64
    /home/fds/dhcam_install_20181107/dh_camera/daheng-sdk-x64/sdk/genicam/bin/Linux64_x64/GenApi/Generic

)

add_executable(daheng src/daheng.cpp)
target_link_libraries(daheng ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
GCBase_gcc40_v2_3 
GenApi_gcc40_v2_3 
log4cpp_gcc40_v2_3 
Log_gcc40_v2_3 
MathParser_gcc40_v2_3
pthread
gxiapi
dximageproc
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
