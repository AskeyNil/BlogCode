cmake_minimum_required(VERSION 3.0.0)
project(Main VERSION 0.1.0)

add_compile_options(-std=c++11)

include(CTest)
enable_testing()


set(OpenCV_DIR /home/askeynil/devtools/libs/opencv-3.4.7/share/OpenCV)
find_package(OpenCV REQUIRED)
# 打印当前 OpenCV 版本和位置
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


add_executable(Main main.cc)
add_subdirectory(cc)
add_subdirectory(FeatureMatching)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
