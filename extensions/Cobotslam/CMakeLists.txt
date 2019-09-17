cmake_minimum_required(VERSION 3.1.0)
project(Cobotslam)
#编译器要支持C++11
set(CMAKE_CXX_STANDARD 11)

set(BUILD_SHARED_LIBS ON)
#用IDE调试的时候可以用这个或者FORCE DEBUG
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")

#引入GMock
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cartographer/cmake/modules/")
#find_package(GMock REQUIRED)

#message(STATUS ${GMock_VERSION})

#直接使用源文件的配置文件而不是安装后的配置文件
set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY "${CMAKE_SOURCE_DIR}/cartographer/configuration_files"
  CACHE PATH ".lua config file directory")

# 编译cartographer
add_subdirectory(cartographer)


#引入编译后的cartographer，find_package(cartographer)，编译cartoslam
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# 编译Cartoslam
add_subdirectory(Cartoslam)

#动态链接库直接拷贝过去用就行，不用安装

