cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Use the system OpenCV, not the one installed in /usr/local:
# - the /usr/local build (4.15.0-dev) has no GUI toolkit, so namedWindow() throws
# - cv_bridge links the system OpenCV, so this avoids loading two cores at once
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(bev scripts/bev.cpp)

# Link OpenCV
target_include_directories(bev PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bev ${OpenCV_LIBRARIES})

# Link ROS 2 dependencies
ament_target_dependencies(bev cv_bridge sensor_msgs)

# Fixed: Added closing parenthesis
install(TARGETS
  bev
  DESTINATION lib/${PROJECT_NAME}
)

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