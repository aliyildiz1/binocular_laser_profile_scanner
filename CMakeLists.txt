cmake_minimum_required(VERSION 3.0.2)
project(binocular_laser_profile_scanner)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    cv_bridge
    std_msgs
    geometry_msgs
    urdf
    xacro
    message_generation
)

catkin_package(CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    cv_bridge
    std_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf scripts)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)