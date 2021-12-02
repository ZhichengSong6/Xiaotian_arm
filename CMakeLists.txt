cmake_minimum_required(VERSION 3.5)
project(XiaotianArm)

set(BUILD_TYPE_RELEASE TRUE)
# set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
# -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
# -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
# -Wshadow -Wsign-promo")
set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -march=native -mavx -mfma -ffast-math -fopenmp")

set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(THIS_COM ${CMAKE_CURRENT_SOURCE_DIR}/)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
          ${CMAKE_BINARY_DIR}/Configuration.h)

message("THIS_COM: " ${THIS_COM})
message("Current cmake binary dir: " ${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(sim)

