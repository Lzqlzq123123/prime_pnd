cmake_minimum_required(VERSION 3.5)
project(noitom_mocap)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
include_directories(third_party/noitom/include/mocapapi)

# set noitom lib path, select x64 or aarch64
set(NOITOM_SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/third_party/noitom")
set(NOITOM_PLATFORM "linux")
set(NOITOM_ARCH "auto" CACHE STRING "Noitom SDK arch: auto|x64|aarch64")
set_property(CACHE NOITOM_ARCH PROPERTY STRINGS auto x64 aarch64)

if(NOITOM_ARCH STREQUAL "auto")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(NOITOM_ARCH_DIR "aarch64")
  else()
    set(NOITOM_ARCH_DIR "x64")
  endif()
else()
  set(NOITOM_ARCH_DIR "${NOITOM_ARCH}")
endif()

set(NOITOM_LIB_DIR "${NOITOM_SDK_ROOT}/bin/${NOITOM_PLATFORM}/${NOITOM_ARCH_DIR}")
if(NOT EXISTS "${NOITOM_LIB_DIR}")
  message(FATAL_ERROR "Noitom lib dir not found: ${NOITOM_LIB_DIR} (set -DNOITOM_ARCH=x64|aarch64 if needed)")
endif()
link_directories("${NOITOM_LIB_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/noitom_data_handler.cpp)
ament_target_dependencies(${CMAKE_PROJECT_NAME} 
  rclcpp
  tf2_ros
  geometry_msgs
)

target_link_libraries(noitom_mocap 
  MocapApi
)

install(TARGETS
  ${CMAKE_PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY "${NOITOM_LIB_DIR}/"
  DESTINATION lib
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
