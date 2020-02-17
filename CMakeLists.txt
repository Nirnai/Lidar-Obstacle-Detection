cmake_minimum_required(VERSION 3.4)

project(ObstacleDetection LANGUAGES CXX)

find_package(PCL 1.2 REQUIRED)
find_package(OpenMP)


add_executable(environment src/environment.cpp src/render.cpp) 
target_include_directories(environment PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(environment PUBLIC ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(environment PUBLIC OpenMP::OpenMP_CXX)
endif()
target_compile_features(environment PUBLIC cxx_std_11)