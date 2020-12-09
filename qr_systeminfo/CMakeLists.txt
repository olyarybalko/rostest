cmake_minimum_required(VERSION 3.0.2)
project(qr_systeminfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rostest
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/point.cpp src/systeminfo.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/point.cpp src/systeminfo.cpp src/rostest_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

catkin_add_gtest(${PROJECT_NAME}-test src/point.cpp test/test_primer.cpp)

target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${PROJECT_NAME})


catkin_add_gtest(${PROJECT_NAME}-pubtest src/point.cpp test/test_publisher.cpp)

target_link_libraries(${PROJECT_NAME}-pubtest ${catkin_LIBRARIES} ${PROJECT_NAME})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
