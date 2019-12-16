cmake_minimum_required(VERSION 3.5)
project(minirys_drivers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# including headers
install(
        DIRECTORY include/
        DESTINATION include
)

# setting ament dependencies
set(AMENT_DEPENDENCIES
	rclcpp
)

# building and installing libraries
set(SOURCE_FILES
	src/FrequencyCounter/FrequencyCounter.cpp
  src/bcm/bcm2835.c
  src/l6470/l6470.cpp
  src/l6470/l6470commands.cpp
  src/l6470/l6470config.cpp
  src/l6470/l6470dump.cpp
  src/l6470/l6470support.cpp
  src/l6470/motors.cpp
  src/lsm6ds3/LSM6DS3.cpp
  src/lsm6ds3/filter.cpp
  src/vl53l1x/VL53L1X.cpp
)

add_library(${PROJECT_NAME}_source ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}_source ${AMENT_DEPENDENCIES})

target_include_directories(${PROJECT_NAME}_source
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
	${PROJECT_NAME}_source
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# building and installing executables
add_executable(frequency_test src/freqmain.cpp)
target_link_libraries(frequency_test ${PROJECT_NAME}_source)
ament_target_dependencies(frequency_test ${AMENT_DEPENDENCIES})

target_include_directories(frequency_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME}_source)
ament_target_dependencies(main ${AMENT_DEPENDENCIES})

target_include_directories(main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


install(TARGETS
	frequency_test
  main
	DESTINATION lib/${PROJECT_NAME}
)

# installing python scripts
ament_python_install_package(scripts/tmp102)
install(PROGRAMS
  scripts/tmp102/tmp102.py
  scripts/tmp102/example.py
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
