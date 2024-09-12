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

# Qt 경로 설정
# To Do : relative path
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/Qt6/lib/cmake/Qt6;${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/Qt6/lib/cmake")
find_package(Qt6 REQUIRED COMPONENTS Widgets Core Gui OpenGL OpenGLWidgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# find_package(OpenGL REQUIRED)

FILE(GLOB SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/QT/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/ROS2/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/OpenGL/*.cpp"

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

ament_target_dependencies(gui rclcpp)

target_link_libraries(gui
        Qt6::OpenGLWidgets
        Qt6::Widgets
        Qt6::OpenGL
        Qt6::Core
        Qt6::Gui
        #OpenGL::GL
)

target_include_directories(gui PRIVATE ${HEADERS})

install(TARGETS
        gui
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()