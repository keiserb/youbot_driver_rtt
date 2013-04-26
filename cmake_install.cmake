# Install script for directory: /home/keiserb/rosstacks/youbot_hardware/youbot_driver_rtt

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/libyoubot_driver_rtt-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/libyoubot_driver_rtt-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/libyoubot_driver_rtt-gnulinux.so"
         RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/youbot_driver_rtt:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/geometry/tf/lib:/opt/ros/fuerte/stacks/bullet/lib:/home/keiserb/rosstacks/soem/soem_core/lib:/home/keiserb/rosstacks/youbot_hardware/rtt_youbot_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_nav_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/keiserb/rosstacks/motion_control/rtt_motion_control_msgs/lib/orocos/gnulinux/types")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt" TYPE SHARED_LIBRARY FILES "/home/keiserb/rosstacks/youbot_hardware/youbot_driver_rtt/lib/orocos/gnulinux/libyoubot_driver_rtt-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/libyoubot_driver_rtt-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/libyoubot_driver_rtt-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/libyoubot_driver_rtt-gnulinux.so"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/geometry/tf/lib:/opt/ros/fuerte/stacks/bullet/lib:/home/keiserb/rosstacks/soem/soem_core/lib:/home/keiserb/rosstacks/youbot_hardware/rtt_youbot_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_nav_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/keiserb/rosstacks/motion_control/rtt_motion_control_msgs/lib/orocos/gnulinux/types:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/youbot_driver_rtt:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/geometry/tf/lib:/opt/ros/fuerte/stacks/bullet/lib:/home/keiserb/rosstacks/soem/soem_core/lib:/home/keiserb/rosstacks/youbot_hardware/rtt_youbot_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_nav_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/keiserb/rosstacks/motion_control/rtt_motion_control_msgs/lib/orocos/gnulinux/types")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/libyoubot_driver_rtt-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins/libyoubot-plugin-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins/libyoubot-plugin-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins/libyoubot-plugin-gnulinux.so"
         RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/plugins:/usr/local/lib/orocos/gnulinux/youbot_driver_rtt/plugins:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/geometry/tf/lib:/opt/ros/fuerte/stacks/bullet/lib:/home/keiserb/rosstacks/soem/soem_core/lib:/home/keiserb/rosstacks/youbot_hardware/rtt_youbot_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_nav_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/keiserb/rosstacks/motion_control/rtt_motion_control_msgs/lib/orocos/gnulinux/types")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins" TYPE SHARED_LIBRARY FILES "/home/keiserb/rosstacks/youbot_hardware/youbot_driver_rtt/lib/orocos/gnulinux/plugins/libyoubot-plugin-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins/libyoubot-plugin-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins/libyoubot-plugin-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins/libyoubot-plugin-gnulinux.so"
         OLD_RPATH "/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/geometry/tf/lib:/opt/ros/fuerte/stacks/bullet/lib:/home/keiserb/rosstacks/soem/soem_core/lib:/home/keiserb/rosstacks/youbot_hardware/youbot_driver_rtt:/home/keiserb/rosstacks/youbot_hardware/rtt_youbot_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_nav_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/keiserb/rosstacks/motion_control/rtt_motion_control_msgs/lib/orocos/gnulinux/types:::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/plugins:/usr/local/lib/orocos/gnulinux/youbot_driver_rtt/plugins:/opt/ros/fuerte/lib:/opt/ros/fuerte/stacks/orocos_toolchain/ocl/lib:/opt/ros/fuerte/stacks/orocos_toolchain/log4cpp/../install/lib:/opt/ros/fuerte/stacks/geometry/tf/lib:/opt/ros/fuerte/stacks/bullet/lib:/home/keiserb/rosstacks/soem/soem_core/lib:/home/keiserb/rosstacks/youbot_hardware/rtt_youbot_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/orocos_toolchain/install/lib:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/plugins:/home/keiserb/myproject/rtt_ros_integration/rtt_rosnode/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_ros_comm/rtt_std_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_geometry_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_nav_msgs/lib/orocos/gnulinux/types:/opt/ros/fuerte/stacks/rtt_common_msgs/rtt_sensor_msgs/lib/orocos/gnulinux/types:/home/keiserb/rosstacks/motion_control/rtt_motion_control_msgs/lib/orocos/gnulinux/types")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/youbot_driver_rtt/plugins/libyoubot-plugin-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/keiserb/rosstacks/youbot_hardware/youbot_driver_rtt/youbot_driver_rtt-gnulinux.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/keiserb/rosstacks/youbot_hardware/youbot_driver_rtt/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/keiserb/rosstacks/youbot_hardware/youbot_driver_rtt/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
