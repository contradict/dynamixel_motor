cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_driver)

find_package(catkin REQUIRED
        )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
)

