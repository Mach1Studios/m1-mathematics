cmake_minimum_required(VERSION 3.25)
project(m1_mathematics)

set(CMAKE_CXX_STANDARD 17)

enable_testing()

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME}
        PUBLIC

        include/m1_mathematics/MathUtility.h
        include/m1_mathematics/Float3.h
        include/m1_mathematics/Orientation.h
        include/m1_mathematics/Quaternion.h

        src/Quaternion.cpp
        src/Orientation.cpp
        src/Float3.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        )

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/f8d7d77c06936315286eb55f8de22cd23c188571.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(${PROJECT_NAME}_tests
        tests/main.cpp

        tests/Float3Tests.cpp
        tests/OrientationTests.cpp
        tests/QuaternionTests.cpp
        )

target_link_libraries(${PROJECT_NAME}_tests
        PRIVATE
        GTest::gtest_main
        m1_mathematics
        )

# For MacOS M1+ ARM targets
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)
endif()

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_tests)
