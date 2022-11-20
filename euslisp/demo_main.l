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
(ros::load-ros-package "topic_tools")
(ros::load-ros-package "speech_recognition_msgs")

(defvar *camera-frame* "head_camera_rgb_optical_frame")
(defvar *base-frame* "base_link")
(defvar *head-camera-points* "/head_camera/depth_registered/points")
(setq *head-camera-img* "/head_camera/rgb/image_rect_color")

;; global variables
(setq *prev-mux-topic* "/speech_to_text_google")
(setq *look-at-pos* #f(500 0 1100))
(setq *torso-z* 50)
(setq *deco-loop-num* 0)
(setq *find-rot-num* 4)
(setq *HUMAN-WILL-VAL* 0.5)  ;; 0~1

(defun init-global-param ()
  (setq pre_x -1)
  (setq *dimg-req-msg* (instance deco_with_fetch::DecoImgsRequest :init))
  (setq *status-req-msg* (instance deco_with_fetch::DecoStatusRequest :init))
  (setq *wall-pcl* nil)
  (setq *wall-x* nil)
  (setq *bboxes* '())
  (setq *place-pos-lst* '())
  (setq *place-uv-pos-lst* '())
  (setq *catch-pos-lst* '())
  (setq *close-deco-x* 0)  ;; to go-back default position
  (setq *catch-box-num* 0)
  (setq *pass-to-human-flag* nil)
  (setq *instruct-msg* (instance deco_with_fetch::InstructInfo :init))
  (send *instruct-msg* :called_count -1)
  (send *instruct-msg* :is_finish t)
  (setq *deco-result-lst* '())
  )

;; load files
(load "common_func.l")
(load "recognize_wall.l")
(load "recognize_decoration.l")
(load "think_decoration.l")
(load "catch_balloon.l")
(load "place_balloon.l")
(load "check_decoration.l")
(load "interact_with_human.l")
(load "instruct_robot_to_human.l")

;; (load "test_manip_balloon.l")

(defun stop-default-mux ()
  (let* ((mux_req_msg (instance topic_tools::MuxSelectRequest :init)))
    (send mux_req_msg :topic "/speech_to_text_julius")
    (ros::wait-for-service "/speech_to_text_mux/select")
    (setq *prev-mux-topic*
	  (send (ros::service-call "/speech_to_text_mux/select" mux_req_msg) :prev_topic))
    (format t "change from ~A to /speech_to_text_julius~%" *prev-mux-topic*)
    ))

(defun set-default-mux ()
  (let* ((mux_req_msg (instance topic_tools::MuxSelectRequest :init))
	 (prev_topic))
    (send mux_req_msg :topic *prev-mux-topic*)
    (ros::wait-for-service "/speech_to_text_mux/select")
    (ros::service-call "/speech_to_text_mux/select" mux_req_msg)
    (format t "change to ~A~%" *prev-mux-topic*)
    ))

(defun main-once ()
  (send *ri* :stop-grasp)
  (make-reset-pose)
  (format t "set reset pose~%")
  (recognize-wall)
  (recognize-decoration)
  (think-decoration)
  (catch-balloon)
  (place-balloon)
  (check-decoration)
  (if (<= *HUMAN-WILL-VAL* 0.5)
      (interact-robot-to-human))
  )

(defun set-sim ()
  (setq *head-camera-img* "/head_camera/rgb/image_raw")
  (setq *PUSH-X-OFFSET* 600)
  (setq *find-rot-num* 5)
  (setq *rotate-thresh* (deg2rad 20))
  (setq *wall-rotate-rad* (deg2rad 20))
  (defun go-vel-rotate (move_rad)  ;; move_rad[rad] move_time[s]
    (if (< move_rad 0)
	(send *ri* :go-velocity 0 0 (- move_rad 0.18))
      (send *ri* :go-velocity 0 0 move_rad)
      )
    (send *ri* :wait-interpolation)
    )
  )

(defun main (loop_num)
  (let* ((loop_num loop_num))
    (dotimes (i loop_num)
      (init-global-param)
      (main-once)
      (setq *deco-loop-num* (+ *deco-loop-num* 1)))
  ))

(defun main-sim (loop_num)
  (let* ((loop_num loop_num))
    (set-sim)
    (main loop_num)
    ))
