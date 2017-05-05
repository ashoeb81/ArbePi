cmake_minimum_required(VERSION 3.0)

project(ArbeRadar)

find_package(WiringPi REQUIRED)
find_package(OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(headers)

file(GLOB SOURCES "src/*.cpp")

add_executable(ArbeRadar ${SOURCES})
target_link_libraries(ArbeRadar ${OpenCV_LIBS})
target_link_libraries(ArbeRadar ${WIRINGPI_LIBRARIES})
