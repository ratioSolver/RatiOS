cmake_minimum_required(VERSION 3.8)
project(deliberative_tier)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS aerials roscpp message_generation)

## Declare ROS messages
add_message_files(FILES
  "Flaw.msg"
  "Graph.msg"
  "Position.msg"
  "Rational.msg"
  "Resolver.msg"
  "Timelines.msg"
)

## Declare ROS services
add_service_files(FILES
  "StateProvider.srv"
)

generate_messages()

add_subdirectory(extern/plexa)

add_executable(${PROJECT_NAME} src/main.cpp src/deliberative_manager.cpp src/deliberative_executor.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${catkin_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)  # Require C++17
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} PlExA oRatioSolver ratioCore RiDDLe SeMiTONE json)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_package()
