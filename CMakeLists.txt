cmake_minimum_required(VERSION 3.8)
project(j7s-rostrace)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

add_executable(j7s-rostrace src/j7s-rostrace.cpp)
target_include_directories(j7s-rostrace PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(j7s-rostrace PUBLIC c_std_99 cxx_std_17)
target_link_libraries(j7s-rostrace fmt::fmt)
ament_target_dependencies(j7s-rostrace rclcpp)

install(TARGETS j7s-rostrace
  DESTINATION lib/${PROJECT_NAME})

ament_package()
