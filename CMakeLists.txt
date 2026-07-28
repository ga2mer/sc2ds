cmake_minimum_required(VERSION 3.15)
project(sc2ds)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(USBIPDCPP_DIR usbipdcpp)

set(USBIPDCPP_USE_PKGCONF_ASIO ON CACHE BOOL "" FORCE)
set(USBIPDCPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(USBIPDCPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(${USBIPDCPP_DIR})

if (NOT TARGET usbipdcpp::usbipdcpp)
  add_library(usbipdcpp::usbipdcpp ALIAS usbipdcpp)
endif ()
if (TARGET usbipdcpp_virtual_device AND NOT TARGET usbipdcpp::virtual_device)
  add_library(usbipdcpp::virtual_device ALIAS usbipdcpp_virtual_device)
endif ()
if (TARGET usbipdcpp_libusb AND NOT TARGET usbipdcpp::libusb)
  add_library(usbipdcpp::libusb ALIAS usbipdcpp_libusb)
endif ()

add_executable(${PROJECT_NAME}
  main.cpp
  UacVirtualInterfaceHandler.cpp
  DualsenseHandler.cpp
)

target_link_libraries(${PROJECT_NAME} 
  usbipdcpp::usbipdcpp
  usbipdcpp::libusb
  usbipdcpp::virtual_device
  fmt
  # asound
)