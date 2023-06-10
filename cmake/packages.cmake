list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${PROJECT_SOURCE_DIR}/src/)

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# eigen 3
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/eigen340)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
)
include_directories(${catkin_INCLUDE_DIRS})


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

set(third_party_libs
        ${catkin_LIBRARIES}
        ${Pangolin_LIBRARIES}
        glog gflags
)