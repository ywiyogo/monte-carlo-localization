cmake_minimum_required( VERSION 3.21 )
project(MCLocalization VERSION 1.0 DESCRIPTION "Playing around the SLAM algorithm in C++" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(mcl src/main.cpp)

set_target_properties(mcl PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(mcl PRIVATE python3.10)

target_include_directories(mcl PUBLIC 
${CMAKE_SOURCE_DIR}/include
/usr/include/python3.10
/usr/lib/python3/dist-packages/numpy/core/include/numpy/
)

# enable_testing()    # it is enough to write this function once in this root file to include test cases in CTest
# add_subdirectory(tests)