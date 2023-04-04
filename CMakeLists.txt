cmake_minimum_required(VERSION 3.0.2)
project(mecanum_control)

find_package(catkin REQUIRED COMPONENTS
        controller_interface
        realtime_tools
        roscpp
)

set(mecanum_drive_controller_sources
        src/mecanum_drive_controller.cpp
        src/mecanum_drive_kinematic.cpp)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mecanum_drive_controller
        CATKIN_DEPENDS controller_interface realtime_tools
)

include_directories(
        include ${catkin_INCLUDE_DIRS}
)

add_library(mecanum_drive_controller ${mecanum_drive_controller_headers} ${mecanum_drive_controller_sources})
target_link_libraries(mecanum_drive_controller ${catkin_LIBRARIES})

install(TARGETS mecanum_drive_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES mecanum_drive_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
        DIRECTORY include/mecanum_drive_controller/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

