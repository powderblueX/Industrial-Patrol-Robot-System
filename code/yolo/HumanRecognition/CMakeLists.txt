project(HumanRecognition)

cmake_minimum_required(VERSION 3.16)

find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(yolo_test main.cpp human_detector.cpp)

target_link_libraries(yolo_test openvino::runtime ${OpenCV_LIBS})