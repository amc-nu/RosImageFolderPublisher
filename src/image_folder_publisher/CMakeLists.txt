cmake_minimum_required(VERSION 2.8.3)
project(image_folder_publisher)

find_package(catkin REQUIRED
        cv_bridge
        sensor_msgs
        rospy
        std_msgs
        )

catkin_python_setup()

catkin_package()

install(PROGRAMS
        scripts/image_folder_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        launch/image_folder_publisher.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )