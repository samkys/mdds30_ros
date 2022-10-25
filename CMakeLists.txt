cmake_minimum_required(VERSION 3.0.2)
project(mdds30_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)



################################################
##       Find required packages               ##
################################################
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
################################################
##       End find required packages           ##
################################################



################################################
##      Declare ROS dynamic reconfigure       ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/mdds30_ros.cfg
)
################################################
##     End Declare ROS dynamic reconfigure    ##
################################################



################################################
##           Catkin includes etc.             ##
################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

################################################
##         End Catkin includes etc.           ##
################################################



################################################
##                  Build                     ##
################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/mdds30_ros.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_node src/mdds30_ros.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
################################################
##                End Build                   ##
################################################



################################################
##                 Install                    ##
################################################
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
################################################
##               End Install                  ##
################################################
