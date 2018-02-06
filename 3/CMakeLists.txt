cmake_minimum_required( VERSION 2.8 )
project( trajectory )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set(Sophus_LIBRARIES /home/song/Desktop/SLAM14/slambook/3rdparty/Sophus/build/libSophus.so)
# sophus
include_directories( /home/song/Desktop/SLAM14/slambook/3rdparty/Sophus )
# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )




# pangolin
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )


# eigen 
include_directories( "/usr/include/eigen3/" )


# add_executable( trajectory draw_trajectory.cpp )
# target_link_libraries( trajectory ${OpenCV_LIBS} ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES}    )



add_executable( rmse rmse.cpp )
target_link_libraries( rmse ${OpenCV_LIBS} ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES}    )
