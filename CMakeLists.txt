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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# including headers
install(
        DIRECTORY include/
        DESTINATION include
)

# setting ament dependencies
set(AMENT_DEPENDENCIES
	rclcpp
  sensor_msgs
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
  src/MotorsController/MotorsController.cpp
  src/vl53l1x_stm/vl53l1_api.c
  src/vl53l1x_stm/vl53l1_api_strings.c
  src/vl53l1x_stm/vl53l1_register_funcs.c
  src/vl53l1x_stm/vl53l1_api_calibration.c
  src/vl53l1x_stm/vl53l1_core.c
  src/vl53l1x_stm/vl53l1_silicon_core.c
  src/vl53l1x_stm/vl53l1_api_core.c
  src/vl53l1x_stm/vl53l1_core_support.c
  src/vl53l1x_stm/vl53l1_wait.c
  src/vl53l1x_stm/vl53l1_api_debug.c
  src/vl53l1x_stm/vl53l1_error_strings.c
  src/vl53l1x_stm/vl53l1_api_preset_modes.c
  src/vl53l1x_stm/vl53l1_platform.c

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

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME}_source)
ament_target_dependencies(main ${AMENT_DEPENDENCIES})

target_include_directories(main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# building and installing executables
add_executable(frequency_test src/freqmain.cpp)
target_link_libraries(frequency_test ${PROJECT_NAME}_source)
ament_target_dependencies(frequency_test ${AMENT_DEPENDENCIES})

target_include_directories(frequency_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(main_old src/main_old.cpp)
target_link_libraries(main_old ${PROJECT_NAME}_source)
ament_target_dependencies(main_old ${AMENT_DEPENDENCIES})

target_include_directories(main_old
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(imu_sensor_test src/imu_sensor_test.cpp)
target_link_libraries(imu_sensor_test ${PROJECT_NAME}_source)
ament_target_dependencies(imu_sensor_test ${AMENT_DEPENDENCIES})

target_include_directories(imu_sensor_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(tof_sensor_test src/tof_sensor_test.cpp)
target_link_libraries(tof_sensor_test ${PROJECT_NAME}_source)
ament_target_dependencies(tof_sensor_test ${AMENT_DEPENDENCIES})

target_include_directories(tof_sensor_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(motors_test src/motors_test.cpp)
target_link_libraries(motors_test ${PROJECT_NAME}_source)
ament_target_dependencies(motors_test ${AMENT_DEPENDENCIES})

target_include_directories(motors_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(joycon_response_test src/joycon_response_test.cpp)
target_link_libraries(joycon_response_test ${PROJECT_NAME}_source)
ament_target_dependencies(joycon_response_test ${AMENT_DEPENDENCIES})

target_include_directories(joycon_response_test
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(joycon_control_only src/joycon_control_only.cpp)
target_link_libraries(joycon_control_only ${PROJECT_NAME}_source)
ament_target_dependencies(joycon_control_only ${AMENT_DEPENDENCIES})

target_include_directories(joycon_control_only
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


install(TARGETS
  main
	frequency_test
  main_old
  imu_sensor_test
  tof_sensor_test
  motors_test
  joycon_response_test
  joycon_control_only
	DESTINATION lib/${PROJECT_NAME}
)

# installing python scripts
install(FILES
  scripts/tmp102/tmp102.py
  scripts/mcp3221/mcp3221.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/temperature_sensor_test.py
  scripts/voltage_sensor_test.py
  scripts/temperature_publisher.py
  scripts/voltage_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
