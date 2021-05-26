cmake_minimum_required(VERSION 2.8.3)
project(self_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
        )


## Generate messages in the 'msg' folder
add_message_files(
	#DIRECTORY msg
        FILES
        wheel_speed.msg
        yaw_rate.msg
	x_acc.msg
	y_acc.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
)
