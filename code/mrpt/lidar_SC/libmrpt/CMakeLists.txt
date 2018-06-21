
set(LIB_SRC mrpt.cpp)

#静态链接库
#add_library(libmrpt ${LIB_SRC})

#动态链接库
add_library(libmrpt SHARED ${LIB_SRC})

#指定库输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib) 

set_target_properties(libmrpt PROPERTIES OUTPUT_NAME "mrpt")
