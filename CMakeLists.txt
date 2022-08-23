cmake_minimum_required(VERSION 3.8)
project(deliberative_tier)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aerials REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "msg/Flaw.msg"
  "msg/Graph.msg"
  "msg/Position.msg"
  "msg/Rational.msg"
  "msg/Resolver.msg"
  "msg/Timelines.msg"
  "srv/StateProvider.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

add_subdirectory(extern/plexa)

add_executable(${PROJECT_NAME} src/main.cpp src/deliberative_manager.cpp src/deliberative_executor.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp aerials)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(${PROJECT_NAME} ${cpp_typesupport_target} PlExA oRatio ratioCore RiDDLe SeMiTONE json)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
