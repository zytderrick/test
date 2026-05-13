cmake_minimum_required(VERSION 3.10)

file(GLOB_RECURSE TEST_SRCS ./*_test.cc)

ament_add_gtest(${PROJECT_NAME}_test ${TEST_SRCS})

target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
)


# Install
install(
    DIRECTORY data/
    DESTINATION data/control/test/
)

install(
    TARGETS ${PROJECT_NAME}_test
    DESTINATION bin/
)

enable_testing()
add_test(NAME ${PROJECT_NAME}_ctest COMMAND ${PROJECT_NAME}_test)
