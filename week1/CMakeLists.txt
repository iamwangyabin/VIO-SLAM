cmake_minimum_required(VERSION 3.2)
project(vio)

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(
    ${Eigen3_INCLUDE_DIRS}
)
set(Sophus_DIR  Sophus/build) 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(question_2 question_2.cpp)
target_link_libraries(question_2 ${Sophus_LIBRARIES})
