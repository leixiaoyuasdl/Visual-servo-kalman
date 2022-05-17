cmake_minimum_required(VERSION 3.15)
PROJECT(visual_servo_kalman)
add_definitions(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        tf
        moveit_msgs
        moveit_ros_perception
        moveit_ros_planning_interface
        )
AUX_SOURCE_DIRECTORY(src source_file)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

ADD_EXECUTABLE(kalman_6dof kalman_6dof.cpp ${source_file} )
target_link_libraries(kalman_6dof ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

ADD_EXECUTABLE(setgoal setgoal.cpp ${source_file} )
target_link_libraries(setgoal ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})