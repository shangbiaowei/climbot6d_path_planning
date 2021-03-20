cmake_minimum_required(VERSION 3.1.0)
project(climbot6d)

add_definitions(-std=c++17)

find_package(NLopt REQUIRED)
 
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

if (UNIX)
include_directories(
    #/home/wei/climbot6d/dlib-19.21
    /usr/local/include
    ${NLOPT_INCLUDE_DIR}
)
else()
include_directories(
    #C:/Users/wei/Desktop/test/dlib-19.21
)
endif()

aux_source_directory(./src DIR_SRCS)

if (UNIX)
link_directories(
    #/home/wei/climbot6d/dlib-19.21/build/dlib
    /usr/local/include
)
else()
link_directories(
    #C:/Users/wei/Desktop/test/dlib-19.21/build/dlib/Release
)
endif()

#头文件目录
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

add_executable(climbot6d ${DIR_SRCS})


#set(pathToDlib /home/wei/climbot6d/dlib-19.21/)
#include(${pathToDlib}/dlib/cmake)

#target_link_libraries(climbot6d dlib::dlib)
target_link_libraries(climbot6d ${NLOPT_LIBRARIES})
