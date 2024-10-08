cmake_minimum_required(VERSION 3.5)
project(carry_along_car_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

## Sources
## Add serial library
add_library(serial SHARED
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if(APPLE) # macOS
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_sources(serial PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_osx.cc
    )
     target_link_libraries(serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # .*nix
    target_sources(serial PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_linux.cc
     )
         target_link_libraries(serial rt pthread)
elseif(WIN32) # Windows
    target_sources(serial PRIVATE
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc
    )
      target_link_libraries(serial setupapi)
    ament_export_libraries(setupapi)
endif()
## Include headers
target_include_directories(serial PRIVATE include)


# 生成rs232.cpp的可执行文件
add_executable(rs232_driver src/rs232.cpp)
target_include_directories(rs232_driver PRIVATE include)
ament_target_dependencies(rs232_driver rclcpp geometry_msgs)
target_link_libraries(rs232_driver serial)

add_executable(rs485_driver src/rs485.cpp)
target_include_directories(rs485_driver PRIVATE include)
ament_target_dependencies(rs485_driver rclcpp geometry_msgs)
target_link_libraries(rs485_driver serial)

install(TARGETS
  serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  rs232_driver
  rs485_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
DESTINATION include/serial
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
