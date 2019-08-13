cmake_minimum_required(VERSION 2.8.3)
project(bigbot_camera)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
      sensor_msgs
      dynamic_reconfigure
 #     driver_base
      cv_bridge
      image_transport
      camera_info_manager
      tf)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/Camera.cfg
    )

catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      sensor_msgs
      dynamic_reconfigure
 #     driver_base
      cv_bridge
      image_transport
      camera_info_manager
      tf
      LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(mycamera src/camera.cpp)
add_dependencies(mycamera ${PROJECT_NAME}_gencfg)
target_link_libraries(mycamera ${catkin_LIBRARIES}  ${OpenCV_LIBS})

message("opencv include dirs = ${OpenCV_INCLUDE_DIRS}" )
message("opencv libs = ${OpenCV_LIBS}")





#install(TARGETS camera camera_timer camera_polling camera_stereo homography
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

foreach(dir bag config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
