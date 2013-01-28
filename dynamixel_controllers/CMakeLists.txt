cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_controllers)

find_package(catkin REQUIRED
        COMPONENTS
            message_generation
            std_msgs
            dynamixel_msgs
        )

# services
add_service_files(
    DIRECTORY srv
    FILES
        RestartController.srv
        SetComplianceSlope.srv
        StartController.srv
        SetComplianceMargin.srv
        SetSpeed.srv
        StopController.srv
        SetCompliancePunch.srv
        SetTorqueLimit.srv
        TorqueEnable.srv
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES
        std_msgs
        dynamixel_msgs
)

catkin_package(
    CATKIN_DEPENDS
        rospy
)

install(
    PROGRAMS
       nodes/controller_manager.py
       nodes/controller_spawner.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
