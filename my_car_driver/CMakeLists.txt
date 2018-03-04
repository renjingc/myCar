cmake_minimum_required(VERSION 2.8.3)
project(my_car_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  my_car_msgs
  hardware_interface
  controller_manager
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs#   visualization_msgs#   my_car_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_car_driver
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp rospy std_msgs tf visualization_msgs my_car_msgs hardware_interface controller_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(my_car_driver
#   src/${PROJECT_NAME}/my_car_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(my_car_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(robothw src/main.cpp src/SerialPort.cpp src/tank_hw.cpp)
#add_executable(datalog src/datalog.cpp)

add_executable(carDriver src/main.cpp src/SerialPort.cpp src/carDriver.cpp)
add_executable(joystick_driver src/joystick.cpp)

add_executable(gpsToOdom src/gpsToOdom.cpp)
add_executable(dataSave src/datalog.cpp)
add_executable(dataSave2 src/datalog2.cpp)
add_executable(odomScanPub src/odomScanPub.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(my_car_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 #target_link_libraries(robothw
 #  ${catkin_LIBRARIES}
 #)
 
 #target_link_libraries(datalog
 #  ${catkin_LIBRARIES}
 #)
 
  target_link_libraries(carDriver
   ${catkin_LIBRARIES}
 )
   target_link_libraries(joystick_driver
   ${catkin_LIBRARIES}
 )

   target_link_libraries(gpsToOdom
   ${catkin_LIBRARIES}
 )
 
    target_link_libraries(dataSave
   ${catkin_LIBRARIES}
 )
     target_link_libraries(dataSave2
   ${catkin_LIBRARIES}
 )
     target_link_libraries( odomScanPub
   ${catkin_LIBRARIES}
 )
