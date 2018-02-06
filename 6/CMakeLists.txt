cmake_minimum_required( VERSION 2.8 )
project( orb )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen
include_directories( "/usr/include/eigen3/" )
include_directories( "/usr/include/boost/" )


# pangolin
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )

# sophus
set(Sophus_LIBRARIES /home/song/Desktop/SLAM14/slambook/3rdparty/Sophus/build/libSophus.so)
include_directories( /home/song/Desktop/SLAM14/slambook/3rdparty/Sophus )



add_executable( optical_flow optical_flow.cpp )
target_link_libraries( optical_flow ${OpenCV_LIBS} )


add_executable( direct_method direct_method.cpp  )
target_link_libraries( direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
