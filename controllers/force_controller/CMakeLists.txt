cmake_minimum_required(VERSION 2.8.3)
project(force_controller)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 geometry_msgs
 sensor_msgs
 baxter_core_msgs
 kdl_parser
 orocos_kdl
 message_generation
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv 
  FILES
  forceControl.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
   baxter_core_msgs
   geometry_msgs
   sensor_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS
	baxter_core_msgs
	geometry_msgs
	message_runtime
	sensor_msgs
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(mykinematics src/kinematics.cpp)
target_link_libraries(mykinematics ${catkin_LIBRARIES})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES} mykinematics)

add_dependencies(mykinematics ${catkin_EXPORTED_TARGETS})


# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
