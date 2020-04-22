cmake_minimum_required(VERSION 2.8.3)
project(amcl_msgs)

find_package(catkin REQUIRED
        COMPONENTS
            nav_msgs
            std_srvs
            message_generation
            std_msgs
            geometry_msgs
        )


# messages
add_message_files(
    DIRECTORY msg
    FILES
    WeightedParticlecloud.msg
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
  CATKIN_DEPENDS nav_msgs std_srvs std_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})