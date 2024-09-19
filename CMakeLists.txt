cmake_minimum_required(VERSION 3.0)
project(robotics_gui)


if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Building in Release mode")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building in Debug mode")
endif()

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Qt
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/Qt6/lib/cmake/Qt6;${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/Qt6/lib/cmake;${CMAKE_PREFIX_PATH}")
find_package(Qt6 REQUIRED COMPONENTS Widgets Core Gui OpenGL OpenGLWidgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Assimp
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/root/share/code/robotics_gui/3rdParty/assimp/lib/cmake/assimp-5.4")
find_package(assimp REQUIRED)


FILE(GLOB SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/Graphics/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/Utils/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/QT/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/ROS2/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

FILE(GLOB MOC_HEADERS
        "${CMAKE_CURRENT_SOURCE_DIR}/include/QT/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/ROS2/*.h"
)


set(HEADERS
        "${CMAKE_CURRENT_SOURCE_DIR}/include/"
        "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/glm-1.0.1/glm"
)

set(RESOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/resources.qrc"
)

add_executable(gui ${SOURCES} ${MOC_HEADERS} ${RESOURCES})

# OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    target_compile_options(gui PRIVATE ${OpenMP_CXX_FLAGS})
    set(OPENMP_LIBRARY ${OpenMP_CXX_LIBRARIES})
else()
    set(OPENMP_LIBRARY)
endif ()


target_include_directories(gui PRIVATE ${HEADERS} ${ASSIMP_INCLUDE_DIR})
target_link_libraries(gui
        Qt6::OpenGLWidgets
        Qt6::Widgets
        Qt6::OpenGL
        Qt6::Core
        Qt6::Gui
        ${OPENMP_LIBRARY}
        ${ASSIMP_LIBRARIES}
)


ament_target_dependencies(gui rclcpp nav_msgs sensor_msgs)

install(TARGETS
        gui
        DESTINATION lib/${PROJECT_NAME}
)


# below option is for ROS
# CLION is not installing. so, currently use fixed shader path
#install(
#        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/shader/
#        DESTINATION lib/${PROJECT_NAME}/shader/
#)
# add_definitions(-DSHADER_DIR="${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/shader")
#
#install(
#        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mesh/
#        DESTINATION lib/${PROJECT_NAME}/mesh/
#)
# add_definitions(-DSHADER_DIR="${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mesh")

# below option is for CLion and ROS. but ROS is limited.
# program source code path is fixed by shader path
# CLION is not installing. so, currently use fixed shader path
add_definitions(-DSHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shader")
add_definitions(-DMESH_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mesh")


ament_package()