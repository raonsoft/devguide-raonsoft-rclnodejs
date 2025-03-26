cmake_minimum_required(VERSION 3.8)
project(node-ros2-demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclnodejs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Install Node.js files
install(DIRECTORY
  src/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.js"
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 