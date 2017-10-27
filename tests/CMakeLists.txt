set(test_sources AllTests.cpp TrajectoryTests.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

link_directories(/usr/local/lib)
link_directories(/usr/lib/x86_64-linux-gnu/)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_library(path_planning_test_lib ${test_sources})
