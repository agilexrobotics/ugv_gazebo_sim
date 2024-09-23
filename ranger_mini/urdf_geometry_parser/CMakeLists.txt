cmake_minimum_required(VERSION 3.0.2)
project(urdf_geometry_parser)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    urdf
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

# URDF geometry parser library
add_library(${PROJECT_NAME}
  include/urdf_geometry_parser/urdf_geometry_parser.h
  src/urdf_geometry_parser.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(urdf_geometry_parser_test
                    test/urdf_geometry_parser.test
                    test/src/urdf_geometry_parser_test.cpp)
  target_link_libraries(urdf_geometry_parser_test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
