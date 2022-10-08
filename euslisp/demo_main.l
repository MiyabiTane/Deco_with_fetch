#!/usr/bin/env roseus

;; init
(ros::roseus "deco_with_robot")

;; robot model
(load "package://fetcheus/fetch-interface.l")
(setq *fetch* (fetch))
(objects (list *fetch*))
(setq *ri* (instance fetch-interface :init))

;; use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

;; load packages
(ros::load-ros-package "sensor_msgs")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")
(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "deco_with_fetch")

(defvar *camera-frame* "head_camera_rgb_optical_frame")
(defvar *base-frame* "base_link")
(defvar *head-camera-points* "/head_camera/depth_registered/points")

;; global variables
(setq pre_x -1)
(setq *look-at-pos* #f(500 0 1000))

;; load files
(load "common_func.l")
(load "test_push_balloon.l")

