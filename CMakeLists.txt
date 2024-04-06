cmake_minimum_required(VERSION 3.1)
project(scout_diagnostics VERSION 0.1)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    dynamic_reconfigure
    scout_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

generate_dynamic_reconfigure_options(
  cfg/ScoutDiagnostics.cfg
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/scout_diagnostics/scout_diagnostics.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(scout_diagnostics_node src/scout_diagnostics_node.cpp)
target_link_libraries(scout_diagnostics_node ${PROJECT_NAME} ${catkin_LIBRARIES})