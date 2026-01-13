cmake_minimum_required(VERSION 3.0.2)
project(cone_navigation)

## 编译选项：使用 C++11 标准
add_compile_options(-std=c++11)

## 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)

## 查找 OpenCV 包
find_package(OpenCV REQUIRED)

## 声明 catkin 包依赖
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs cv_bridge image_transport std_msgs
)

## 指定头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## 声明可执行文件：img_node
## [cite_start]你的 main.cpp 中 ros::init 定义节点名为 "img_node" [cite: 462]
add_executable(img_node src/main.cpp)

## 链接库文件
target_link_libraries(img_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)