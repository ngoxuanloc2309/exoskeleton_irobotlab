cmake_minimum_required(VERSION 3.8)
project(tongquan_sldasm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Skip linter which checks for copyrights
  # Comment this line when a copyright and license are added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  
  # Skip cpplint (only works in a git repo)
  # Comment this line when this package is in a git repo and when
  # a copyright and license are added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
