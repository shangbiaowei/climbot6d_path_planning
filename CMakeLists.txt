cmake_minimum_required(VERSION 3.5.0)
project(climbot6d)

add_definitions(-std=c++14)

find_package(NLopt REQUIRED)
find_package(OMPL REQUIRED)
 
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

aux_source_directory(./src DIR_SRCS)


#头文件目录
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

include_directories(
    /usr/local/include
    ${NLOPT_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
)

link_directories(
    /usr/local/include
    ${OMPL_LIBRARY_DIRS}
)
add_definitions(${OMPL_DEFINITIONS})

find_package(Boost COMPONENTS system REQUIRED)

add_executable(climbot6d ${DIR_SRCS})

target_link_libraries(climbot6d ${NLOPT_LIBRARIES} ${OMPL_LIBRARIES})