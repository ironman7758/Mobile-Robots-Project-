cmake_minimum_required(VERSION 3.8)
project(p3at_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment and add further dependencies as needed
# find_package(<dependency> REQUIRED)

# Install directories into <install_prefix>/share/<project_name>
install(
  DIRECTORY
    launch
    meshes
    urdf
    world
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip certain checks if desired
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
