cmake_minimum_required(VERSION 3.0.2)
project(orb_fast_livo)

# 查找Eigen3库
find_package(Eigen3 REQUIRED NO_MODULE)
# 查找PCL
find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# 查找OpenCV
find_package(OpenCV REQUIRED)
find_package(Boost 1.71 REQUIRED)

set(CMAKE_PREFIX_PATH "/opt/ros/noetic" ${CMAKE_PREFIX_PATH})

# 查找Catkin依赖包
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        sensor_msgs
        std_msgs
        geometry_msgs
        message_filters
        tf
)

# 声明Catkin包
catkin_package(
        CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs
        DEPENDS Eigen3
)

# 指定头文件目录
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# 添加可执行文件
 add_executable(main
         src/main.cpp
         src/grab.cpp
         src/ekf.cpp
 )

 # 链接库文件
 target_link_libraries(main
         ${catkin_LIBRARIES}
         Eigen3::Eigen
         ${VTK_LIBRARIES}
         ${PCL_LIBRARIES}
 )

#add_executable(ukf src/ukf.cpp
#)
#
#target_link_libraries(ukf
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen
#
#)
#add_executable(uitls src/utils.cpp)
#
#target_link_libraries(uitls
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen
#        ${PCL_LIBRARIES}
#        ${OpenCV_LIBS}
#)
