cmake_minimum_required(VERSION 3.8)
project(usbl_simulator_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Define a custom flag to switch between nanoAUV and ROS 2 standard messages
option(ENABLE_CUSTOM_NANOAUV_INTERFACES "Enable custom nanoAUV driver interfaces" ON)

# Initialize the optional dependency variable
set(OPTIONAL_DEPENDENCIES "")

if(ENABLE_CUSTOM_NANOAUV_INTERFACES)
    # Find nanoauv sensor driver interface package (project-specific)
    find_package(nanoauv_sensor_driver_interfaces REQUIRED)

    # Set variable with dependency for sensor driver interface package
    set(OPTIONAL_DEPENDENCIES nanoauv_sensor_driver_interfaces)

    message(STATUS "Enabled custom nanoAUV driver interfaces")
else()
    message(STATUS "Disabled custom nanoAUV ROS 2 interfaces")
endif()

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR})

set(HEADER_FILES
        include/usbl_simulator.hpp)

add_library(usbl_simulator SHARED
        src/usbl_simulator.cpp
        ${HEADER_FILES})

# add usbl_simulator_node
add_executable(${PROJECT_NAME}_node src/usbl_simulator_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
        rclcpp
        std_msgs
        nav_msgs
        diagnostic_msgs
        tf2_ros
        Eigen3
        ${OPTIONAL_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}_node PUBLIC usbl_simulator)

target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)
 
# Add preprocessor definition for custom nanoAUV sensor driver interfaces macro if enabled
if(ENABLE_CUSTOM_NANOAUV_INTERFACES)
        target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_NANOAUV_SENSOR_DRIVER_INTERFACES)
endif()

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION
        lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION
        share/${PROJECT_NAME}/)

install(TARGETS usbl_simulator
        EXPORT usbl_simulator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips uncrustify (source code formatting)
  # set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES
      test/main.cpp
      test/usbl_simulator_test.cpp)

  ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILES})

  target_link_libraries(${PROJECT_NAME}_test usbl_simulator)

  install(TARGETS
          ${PROJECT_NAME}_test
          DESTINATION lib/${PROJECT_NAME})

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify)

  # enforce linters and static code analyzers defined in ament_lint_common package
  ament_lint_auto_find_test_dependencies()

  # uncomment to include uncrustify explicitly
  # find_package(ament_cmake_uncrustify)
  # ament_uncrustify(CONFIG_FILE "./uncrustify.cfg" TESTNAME "custom_uncrustify")

endif()

ament_package()