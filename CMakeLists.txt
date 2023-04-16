cmake_minimum_required(VERSION 3.10.2)
project(gazebo_noisy_depth_camera)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS gazebo_dev
)

include_directories(
  include
)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/NoisyDepthCameraSensor.cpp
  src/DepthImageGaussianNoiseModel.cpp
  src/MultiplicativeGaussianNoiseModel.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES sensors.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
