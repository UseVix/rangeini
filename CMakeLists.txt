cmake_minimum_required(VERSION 3.8)
project(rangeini)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cloudini_lib REQUIRED)
find_package(hesai_ros_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)
pkg_check_modules(ZSTD REQUIRED libzstd)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedLidarPackets.msg"
  DEPENDENCIES std_msgs
)

add_executable(compressor
  src/Hesai_compressor.cpp
  src/Hesai_compressor_config.cpp
)
target_include_directories(compressor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${LZ4_INCLUDE_DIRS}
  ${ZSTD_INCLUDE_DIRS}
)
target_link_libraries(compressor
  ${LZ4_LIBRARIES}
  ${ZSTD_LIBRARIES}
)

target_compile_features(compressor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  compressor
  "cloudini_lib"
  "hesai_ros_driver"
  "rclcpp"
  "std_msgs"
)

rosidl_target_interfaces(compressor
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(decompressor
  src/Hesai_decompressor.cpp
  src/Hesai_compressor_config.cpp
)
target_include_directories(decompressor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${LZ4_INCLUDE_DIRS}
  ${ZSTD_INCLUDE_DIRS}
)
target_link_libraries(decompressor
  ${LZ4_LIBRARIES}
  ${ZSTD_LIBRARIES}
)

target_compile_features(decompressor PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  decompressor
  "cloudini_lib"
  "hesai_ros_driver"
  "rclcpp"
  "std_msgs"
)

rosidl_target_interfaces(decompressor
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS compressor decompressor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
