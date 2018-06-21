
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
#当前为mrpt中sample样例移植程序
#设置头文件搜索路径
include_directories(${PROJECT_SOURCE_DIR}/include) 	    #这里放公共头文件，依赖库头文件
include_directories(${PROJECT_SOURCE_DIR}/include/mrpt) #MRPT 的头文件 
#include_directories(${PROJECT_SOURCE_DIR}/include/other) #other 的头文件 

include_directories(${PROJECT_SOURCE_DIR}/src/lidar/src) #这里放程序自带头文件，目前无 


	    

#设置库文件寻找路径
link_directories(${PROJECT_SOURCE_DIR}/lib)	
#link_directories(${PROJECT_SOURCE_DIR}/lib/lidar)    
  
  

#设置可执行文件生成路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})	    #生成在根目录下

file(GLOB HEADER_FILES *h)

source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SOURCE_FILES})

#识别目录下的源文件，SOURCE_FILES
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/socket SOURCE_FILES) #该样例程序源文件加入到变量TEST_src中

#用来生成可执行文件
ADD_EXECUTABLE(socket_test ${SOURCE_FILES} ${HEADER_FILES})	#生成执行文件，名为robot

SET_TARGET_PROPERTIES(socket_test PROPERTIES LINKER_LANGUAGE CXX) ##？？？

#链接的库名xxx,xxx是libxxx.so的一部分
#添加链接库名
#target_link_libraries(lidar_test SDL)
#target_link_libraries(lidar_test SDL_image)
#target_link_libraries(lidar_test SDL_ttf)
#target_link_libraries(lidar_test SDL_mixer)
#target_link_libraries(roblidar_test mrpt-nav)

#添加链接库名mrpt
target_link_libraries(socket_test mrpt-base)
target_link_libraries(socket_test mrpt-obs)
target_link_libraries(socket_test mrpt-hwdrivers)
target_link_libraries(socket_test mrpt-maps)
target_link_libraries(socket_test mrpt-slam)
target_link_libraries(socket_test mrpt-opengl)
target_link_libraries(socket_test mrpt-gui)



