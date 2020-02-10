cmake_minimum_required(VERSION 3.1)

project(ObstacleDetection LANGUAGES CXX)

find_package(PCL 1.8 REQUIRED)

add_executable(environment src/environment.cpp src/render.cpp src/PointProcessor.cpp)
target_include_directories(environment PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(environment PUBLIC ${PCL_LIBRARIES})
target_compile_features(environment PUBLIC cxx_std_11)