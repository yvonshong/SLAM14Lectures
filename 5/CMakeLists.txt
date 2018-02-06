cmake_minimum_required( VERSION 2.8 )
project( orb )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )
set(Sophus_LIBRARIES /home/song/catkin_ws/src/slambook/3rdparty/Sophus/build/libSophus.so)
# sophus
include_directories( //home/song/catkin_ws/src/slambook/3rdparty/Sophus)

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen
include_directories( "/usr/include/eigen3/" )

add_executable( computeORB computeORB.cpp )
# 链接OpenCV库
target_link_libraries( computeORB ${OpenCV_LIBS} )

# pangolin
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )


add_executable( BA GN-BA.cpp )
target_link_libraries( BA ${Sophus_LIBRARIES} )


add_executable( E2Rt E2Rt.cpp )
target_link_libraries( E2Rt ${OpenCV_LIBS} )



add_executable( align_trajectory align_trajectory.cpp )
target_link_libraries( align_trajectory  ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES}    )
