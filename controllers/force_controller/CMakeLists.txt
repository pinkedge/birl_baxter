cmake_minimum_required(VERSION 2.8.3)
project(force_controller)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
 roscpp
 geometry_msgs
 sensor_msgs
 baxter_core_msgs
 kdl_parser
 orocos_kdl
 dynamic_reconfigure
 message_generation
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

## Dynamic Reconfigure: takes path to all cfg files taht we want to build.
generate_dynamic_reconfigure_options(cfg/force_error_constants.cfg)

# Generate services in the 'srv' folder
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
	sensor_msgs
	geometry_msgs
  dynamic_reconfigure
	message_runtime
)

include_directories(
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake}
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# Add Libs and Executables
add_library(mykinematics src/kinematics.cpp)
target_link_libraries(mykinematics ${catkin_LIBRARIES})

add_executable(controller src/controller.cpp)
add_dependencies(mykinematics ${catkin_EXPORTED_TARGETS})
add_dependencies(controller %{PROJECT_NAME}_gencfg)
target_link_libraries(controller ${catkin_LIBRARIES} mykinematics)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
