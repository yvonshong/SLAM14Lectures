cmake_minimum_required( VERSION 2.8 )
project( image )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )


# pangolin
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )


# eigen
include_directories( "/usr/include/eigen3/" )


add_executable( imageBasics undistort_image.cpp )
# 链接OpenCV库
target_link_libraries( imageBasics ${OpenCV_LIBS} )

add_executable( disparity disparity.cpp )
target_link_libraries( disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES} )

add_executable( gaussnewton gaussnewton.cpp )
target_link_libraries( gaussnewton ${OpenCV_LIBS}  )


