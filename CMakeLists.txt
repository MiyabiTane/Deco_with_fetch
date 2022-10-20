cmake_minimum_required(VERSION 2.8.3)
project(deco_with_fetch)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs sensor_msgs jsk_recognition_msgs message_generation)

add_service_files(DIRECTORY srv FILES
  DecoImgs.srv
  DecoStatus.srv
  PointStamped.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs jsk_recognition_msgs geometry_msgs
  )

catkin_package()

install(DIRECTORY euslisp launch scripts srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
