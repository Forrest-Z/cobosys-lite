cmake_minimum_required(VERSION 2.8)
project(Cartoslam)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-reorder -O2 -DRRLOG")

set(CMAKE_BUILD_TYPE Release)

#两种类型的文件需用引号分开
file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.h")


#用于编译player中程序
MESSAGE(${CMAKE_CURRENT_LIST_DIR})
add_library(litedriverdef INTERFACE)
target_compile_features(litedriverdef INTERFACE cxx_auto_type)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set(CMAKE_MODULE_PATH ~/catkin_ws/src/cartographer/cmake/modules)

set(cartographer_DIR ~/catkin_ws/install_isolated/share/cartographer)
find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
google_initialize_cartographer_project()
google_enable_testing()

include(FindPkgConfig)


find_package(LuaGoogle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)


INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(~/catkin_ws/src/cartographer)
INCLUDE_DIRECTORIES(~/catkin_ws/build_isolated/cartographer/install)
INCLUDE_DIRECTORIES(/usr/local/include/player-3.1/)
include_directories(/usr/include/lua5.2)
INCLUDE_DIRECTORIES(~/catkin_ws/install_isolated/include)


add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)
target_link_libraries(${PROJECT_NAME} PUBLIC litedriverdef ${LIB_CONFIG})

#使用SYSTEM 添加的库之后需要交叉编译修改
#Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})


#Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC -lglog)


set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
#设置compile_flags
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})


