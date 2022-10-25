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
(setq *look-at-pos* #f(500 0 1100))
(setq *torso-z* 50)
(setq *deco-loop-num* 0)

(defun init-global-param ()
  (setq pre_x -1)
  (setq *dimg-req-msg* (instance deco_with_fetch::DecoImgsRequest :init))
  (setq *status-req-msg* (instance deco_with_fetch::DecoStatusRequest :init))
  (setq *wall-pcl* nil)
  (setq *wall-x* nil)
  (setq *bboxes* '())
  (setq *place-pos-lst* '())
  (setq *catch-pos-lst* '())
  (setq *close-deco-x* 0)  ;; to go-back default position
  (setq *rotate-rad* 0)
  (setq *catch-box-num* 0)
  )

;; load files
(load "common_func.l")
(load "recognize_wall.l")
(load "recognize_decoration.l")
(load "think_decoration.l")
(load "catch_balloon.l")
(load "place_balloon.l")
(load "check_decoration.l")

(load "test_manip_balloon.l")

(defun main-once ()
  (send *ri* :stop-grasp)
  (make-reset-pose)
  (recognize-wall)
  (recognize-decoration)
  (think-decoration)
  (catch-balloon)
  (place-balloon)
  (check-decoration)
  )

(defun main_sim ()
  (setq *head-camera-img* "/head_camera/rgb/image_raw")

  (defun go-vel-rotate (move_rad)  ;; move_rad[rad] move_time[s]
    (if (< move_rad 0)
	(send *ri* :go-velocity 0 0 (- move_rad 0.18))
      (send *ri* :go-velocity 0 0 move_rad)
      )
    (send *ri* :wait-interpolation)
    )

  (dotimes (i 2)
    (init-global-param)
    (main-once)
    (setq *deco-loop-num* (+ *deco-loop-num* 1)))
  )

;; main
(defun main ()
  )
