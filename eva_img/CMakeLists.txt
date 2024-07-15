cmake_minimum_required(VERSION 3.0.2)
project(image_quality_evaluator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_quality_evaluator src/image_quality_evaluator.cpp)
target_link_libraries(image_quality_evaluator ${catkin_LIBRARIES} ${OpenCV_LIBS})
