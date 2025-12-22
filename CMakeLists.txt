cmake_minimum_required(VERSION 3.8)
project(UKF_Server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # add -O3 when you really have to run
endif()


find_package(Eigen3 REQUIRED)

include_directories(
  kalman/include
  /usr/include/eigen3/
)


add_executable(${PROJECT_NAME} src/main.cpp)



install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

