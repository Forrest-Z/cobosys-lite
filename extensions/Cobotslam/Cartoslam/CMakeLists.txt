
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-reorder -O2 -DRRLOG")
#写一个FindPlayer.cmake更合适，先这样写也行
SET (CMAKE_MODULE_PATH "/usr/local/share/cmake/Modules/")
INCLUDE (UsePlayerPlugin)
set(CMAKE_BUILD_TYPE Release)

#两种类型的源文件需用引号分开
file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.h")


#用于编译player中程序
#只在这个路径下查找
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
#添加eigen player头文件
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(/usr/local/include/player-3.1/)

INCLUDE_DIRECTORIES (${PLAYERCORE_EXTRA_INCLUDE_DIRS})
LINK_DIRECTORIES (${PLAYERCORE_EXTRA_LINK_DIRS})

add_library(${PROJECT_NAME} SHARED ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)
target_link_libraries(${PROJECT_NAME} PUBLIC playercore )



