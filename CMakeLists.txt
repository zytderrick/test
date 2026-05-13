cmake_minimum_required(VERSION 3.10)

project(control)
include(Safty)

add_definitions(-DMODULE_NAME="${PROJECT_NAME}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS "-Wno-maybe-uninitialized -Wno-sign-compare -no-pie")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -Wall -Werror -g -ffp-contract=off")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG -Wall -Werror -ffp-contract=off")

if(WITH_COV)
    add_compile_options(-fprofile-arcs -ftest-coverage)
    link_libraries(gcov)
endif()

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(Pavaro REQUIRED)
find_package(OSQP REQUIRED)
find_package(qpOASES REQUIRED)
find_package(pnc_common REQUIRED)

include_directories(
    ../
    ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_LISTS src/*.cc)

add_library(${PROJECT_NAME} SHARED
    ${SRC_LISTS}
)

target_link_libraries(${PROJECT_NAME}
    glog::glog
)

ament_target_dependencies(${PROJECT_NAME}
    Pavaro
    OSQP
    qpOASES
    pnc_common
)

install(
    DIRECTORY src/ 
    DESTINATION include/${PROJECT_NAME}/src
    FILES_MATCHING 
    PATTERN "*.h"
)

install(
    DIRECTORY ${CMAKE_BINARY_DIR}/proto/
    DESTINATION include/${PROJECT_NAME}_proto
    FILES_MATCHING PATTERN "*.h*"
)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib
)

install(
    DIRECTORY dag/
    DESTINATION dag
)

install(
    DIRECTORY module/
    DESTINATION module
)

install(
    DIRECTORY conf/
    DESTINATION conf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(GTest REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(unit_test)
endif()

ament_export_include_directories(
    include/${PROJECT_NAME}
    include/${PROJECT_NAME}_proto
)

ament_export_libraries(
    ${PROJECT_NAME}
    ${PROJECT_NAME}_proto 
)

ament_package()
