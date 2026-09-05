cmake_minimum_required(VERSION 3.16)

project(FlightDynamicsSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(flight_sim
    src/main.cpp
)

target_compile_definitions(flight_sim PRIVATE
    PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(flight_sim PRIVATE Eigen3::Eigen)