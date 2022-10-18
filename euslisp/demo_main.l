#!/usr/bin/env roseus

;; init
(ros::roseus "deco_with_robot")

;; robot model
(load "package://fetcheus/fetch-interface.l")
(setq *fetch* (fetch))
;; (objects (list *fetch*))
;; (setq *ri* (instance fetch-interface :init))
(unless (boundp '*ri*) (setq *ri* (instance fetch-interface :init)))

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
(setq *head-camera-img* "/head_camera/rgb/image_rect_color")

;; global variables
(setq pre_x -1)
(setq *look-at-pos* #f(500 0 1100))
(setq *dimg-req-msg* (instance deco_with_fetch::DecoImgsRequest :init))
(setq *wall-pcl* nil)
(setq *wall-x* nil)
(setq *torso-z* 50)
(setq *bboxes* '())
(setq *place-pos-lst* '())
(setq *catch-pos-lst* '())
(setq *close-deco-x* 0)  ;; to go-back default position
(setq *center-box-num* 0)

;; load files
(load "common_func.l")
(load "recognize_wall.l")
(load "recognize_decoration.l")
(load "think_decoration.l")
(load "catch_balloon.l")
(load "place_balloon.l")

;; (load "test_push_balloon.l")

(defun main_sim ()
  (send *ri* :stop-grasp)
  (setq *head-camera-img* "/head_camera/rgb/image_raw")
  (make-reset-pose)
  (recognize-wall)
  (recognize-decoration)
  (think-decoration)
  (catch-balloon)
  (place-balloon)
  )

;; main
(defun main ()
  (send *ri* :stop-grasp)
  (make-reset-pose)
  ;; (recognize-wall)
  ;; (recognize-decoration)
  ;; (think-decoration)
  )

