cmake_minimum_required(VERSION 2.8.3)
project(ardent_description)

add_compile_options(-std=c++11)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gazebo_ros
  std_msgs
  controller_manager 
  joint_state_controller 
  robot_state_publisher
  genmsg
  actionlib_msgs
  actionlib
  hardware_interface
  urdf
  cmake_modules
)
# find_package(gazebo REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)


link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_NAME}
  ${urdfdom_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} src/hardware_interface.cc)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros
    hardware_interface
    urdf
    cmake_modules
  DEPENDS 
    Boost
    gazebo_ros
    TinyXML
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hardware_interface_test test/hardware_interface_test.cpp)
target_link_libraries(hardware_interface_test ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
