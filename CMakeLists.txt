project(Path_Planning)
cmake_minimum_required (VERSION 3.5)
add_definitions(-std=c++11 -g)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(external)
include_directories(src)

find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})

link_directories(/usr/local/lib)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

add_subdirectory (src) 
add_subdirectory (tests)

add_executable(path_planning src/main.cpp)
target_link_libraries(path_planning path_planning_lib)

add_executable(planner_tests tests/AllTests.cpp)
target_link_libraries(planner_tests path_planning_test_lib path_planning_lib -pthread ${GTEST_LIBRARIES} ${PYTHON_LIBRARIES})

enable_testing()
add_test (NAME TrajectoryTests COMMAND planner_tests)
