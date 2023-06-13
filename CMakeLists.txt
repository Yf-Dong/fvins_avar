cmake_minimum_required(VERSION 2.8.3)
project(fvins)

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -g -rdynamic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--as-needed ${THREADTEST_COMPILE_FLAG}")

find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/thirdpart
${PROJECT_SOURCE_DIR}/thirdpart/yaml-cpp
${PROJECT_SOURCE_DIR}/thirdpart/time/include/fvins
${OpenCV_INCLUDE_DIRS}
)

add_library(ftime
${PROJECT_SOURCE_DIR}/thirdpart/time/src/Time.cc
${PROJECT_SOURCE_DIR}/thirdpart/time/src/Duration.cc
)

add_library(slam SHARED
${PROJECT_SOURCE_DIR}/src/SlamSystem.cc
${PROJECT_SOURCE_DIR}/src/ORBExtractor.cc
${PROJECT_SOURCE_DIR}/src/DescriptorPattern.cc
${PROJECT_SOURCE_DIR}/src/ORBDescriptor.cc
)

link_directories(${PROJECT_SOURCE_DIR}/thirdpart/yaml-cpp)

# add_executable(test examples/exa.cc  examples/exaFE.cc)
# target_link_libraries(test ${OpenCV_LIBRARIES} yaml-cpp ftime)
# target_link_libraries(test ${OpenCV_LIBRARIES} ftime ${OpenCV_LIBRARIES})

add_executable(test2 examples/exaorb.cc)
target_link_libraries(test2 slam ${OpenCV_LIBRARIES} yaml-cpp -lpthread ftime) #dl