cmake_minimum_required(VERSION 2.8)

project(g2o_customBundle)
set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-O3" )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Cholmod REQUIRED)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/ceres) 
include_directories(${PROJECT_SOURCE_DIR}/common
                    ${PROJECT_SOURCE_DIR}/common/tools 
                    ${PROJECT_SOURCE_DIR}/common/flags)

message("find g2o libraries in: " ${G2O_INCLUDE_DIR})
message("find g2o lib in " ${G2O_LIBS})
message("find cholmod in " ${CHOLMOD_INCLUDE_DIR})

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

add_executable(${PROJECT_NAME} g2o_bundle.cpp)

target_link_libraries(${PROJECT_NAME} ${G2O_LIBS} ${CHOLMOD_LIBRARIES} BALProblem ParseCmd)



# pangolin
# set(Pangolin_INCLUDE_DIRS /usr/local/include/pangolin)
# include_directories( /usr/local/include/pangolin )
# set(Pangolin_LIBRARIES  /usr/local/lib/libpangolin.so  )
find_package(Pangolin)
include_directories( ${Pangolin_INCLUDE_DIRS} )
message("find Pangolin in " ${Pangolin_INCLUDE_DIRS})


# 寻找OpenCV库
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# sophus
set(Sophus_LIBRARIES /home/song/catkin_ws/Sophus/build/libSophus.so)
include_directories( /home/song/catkin_ws/Sophus )
message("find Sophus_LIBRARIES in " ${Sophus_LIBRARIES})



add_executable(directBA directBA.cpp)
target_link_libraries(directBA ${G2O_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
