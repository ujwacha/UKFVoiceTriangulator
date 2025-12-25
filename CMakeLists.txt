cmake_minimum_required(VERSION 3.8)
project(UKF_Server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)


include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)


include_directories(
  kalman/include
  /usr/include/eigen3/
  ${CMAKE_CURRENT_SOURCE_DIR}/evpp
  ${CMAKE_CURRENT_SOURCE_DIR}/evpp/evpp
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/evpp/build/lib
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  evpp
  Threads::Threads
  event
  event_core
  glog
  rerun_sdk
)

# Set RPATH so it can find libevpp.so at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/evpp/build/lib"
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/evpp/build/lib"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)
