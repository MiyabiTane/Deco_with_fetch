#!/usr/bin/env roseus

(defvar *bounding-box-list* "/segmentation_decomposer/boxes")

(defun find-boxes ()
  (let* ((box_msg) (box_coords) (box_z))
    (setq *bboxes* '())
    (ros::duration-sleep 1.0)
    (setq bboxes_msg (one-shot-subscribe *bounding-box-list*
					 jsk_recognition_msgs::BoundingBoxArray
					 :after-stamp (ros::time-now)))
    (dotimes (i (length (send bboxes_msg :boxes)))
      (setq box_msg (elt (send bboxes_msg :boxes) i))
      (setq box_coords (elt (bbox->lst box_msg) 0))
      (setq box_z (elt (send box_coords :pos) 2))
      ;; ignore box on the floor
      (if (> box_z 300)
	  (setq *bboxes* (append *bboxes* (list box_msg)))１
	))
    (when (> (length *bboxes*) 0)
      (if (= (send (send (send (elt *bboxes* 0) :pose) :position) :x) 0)
	  (setq *bboxes* '())))
    ))

(defun calc-mean-pos ()
  (let* ((mean_pos #f(0 0 0))
	 (coords_dims_lst) (pos)
	 (box_num (length *bboxes*)))
    (dotimes (i box_num)
      (setq coords_dims_lst (bbox->lst (elt *bboxes* i)))
      (setq pos (send (elt coords_dims_lst 0) :pos))
      (setq mean_pos (v+ mean_pos
			 (float-vector (elt pos 0) (elt pos 1) (elt pos 2))))
      )
    (setq mean_pos (scale (/ 1.0 box_num) mean_pos))
    mean_pos)
  )

(defun get-center-box-pos ()
  (let* ((ypos_lst '())
	 (mean_x 0) (mean_y 0) (mean_z 0)
	 (coords_dims_lst) (pos)
	 (box_num (length *bboxes*)))
    (dotimes (i box_num)
      (setq coords_dims_lst (bbox->lst (elt *bboxes* i)))
      (setq pos (send (elt coords_dims_lst 0) :pos))
      ;; (format t "pos: ~A~%" pos)
      (setq mean_x (+ mean_x (elt pos 0)))
      (setq mean_z (+ mean_z (elt pos 2)))
      (setq ypos_lst (append ypos_lst (list (elt pos 1)))))
    (setq mean_x (/ mean_x box_num))
    (setq mean_z (/ mean_z box_num))
    (setq ypos_lst (sort ypos_lst #'<=))
    (if (= box_num 1)
	(setq mean_y (elt ypos_lst 0))
      (progn
	(if (= (/ (length ypos_lst) 2) (/ (length ypos_lst) 2.0)) ;; even
	    (setq mean_y (/ (+ (elt ypos_lst (- (/ (length ypos_lst) 2) 1))
			       (elt ypos_lst (/ (length ypos_lst) 2)))
			    2.0))
	  (setq mean_y (elt ypos_lst (/ (length ypos_lst) 2)))))
      )
    (format t "center pos: ~A~%" (float-vector mean_x mean_y mean_z))
    (float-vector mean_x mean_y mean_z))
  )

(defun find-decoration (look_at_pos)
  (let* ((deco_count 0)
	 (fin_flag 0)
	 (look_at_pos look_at_pos)
	 (center_y 60))
    (send *fetch* :head :look-at look_at_pos)
    (send-robot 2000)
    (dotimes (i 4)
      (go-vel-rotate (/ pi 4)))
    (while (= fin_flag 0)
      (find-boxes)
      (format t "find ~A boxes~%" (length *bboxes*))
      (when (< (length *bboxes*) deco_count)
	(go-vel-rotate (* -1.0 (/ pi 8)))
	(find-boxes)
	(format t "find ~A boxes~%" (length *bboxes*))
	(setq fin_flag 1))
      (when (= fin_flag 0)
	(setq deco_count (length *bboxes*))
	(go-vel-rotate (/ pi 8)))
      )
    ;; look at center
    (while (> (abs center_y) 50)
      (find-boxes)
      (setq center_y (elt (get-center-box-pos) 1))
      (when (> (abs center_y) 50)
	(if (> center_y 0)
	    (go-vel-rotate (/ pi 10))
	  (go-vel-rotate (* -1 (/ pi 10))))
	))
    (find-boxes)
    (format t "find ~A boxes~%" (length *bboxes*))
    ))

(defun get-decoration-x ()
  (let* ((deco_x 1000))
    (if (> (length *bboxes*) 0)
	(setq deco_x (elt (send (elt (bbox->lst (elt *bboxes* 0)) 0) :pos) 0)))
    (format t "deco_x: ~A~%" deco_x)
    deco_x)
  )

(defun close-decoration ()
  (let* ((fin_flag 0)
	 (deco_count (length *bboxes*))
	 (look_at_pos (get-center-box-pos)))
    (send *fetch* :torso :waist-z :joint-angle 385)
    (send *fetch* :head :look-at look_at_pos)
    (send-robot 1000)
    (while (= fin_flag 0)
      (go-vel-linear (/ 150 1000.0) 1)
      (send *fetch* :head :look-at (v- look_at_pos #f(150 0 0)))
      (send-robot 1000)
      (find-boxes)
      (when (or (< (length *bboxes*) deco_count) (< (get-decoration-x) 1000))
	(go-vel-linear (/ -150 1000.0) 1)
	(send *fetch* :head :look-at look_at_pos)
	(send-robot 1000)
	(ros::duration-sleep 0.5)
	(find-boxes)
	(format t "find ~A boxes, look at: ~A~%" (length *bboxes*) look_at_pos)
	(setq fin_flag 1))
      (when (= fin_flag 0)
	(setq deco_count (length *bboxes*))
	(setq look_at_pos (v- look_at_pos #f(150 0 0)))
	))
    (send *fetch* :head :look-at (get-center-box-pos))
    (send-robot 1000)
    (find-boxes)
    (format t "find ~A boxes~%" (length *bboxes*))
    ))

(defun get-deco-image ()
  (let* ((bbox_array_msg (instance jsk_recognition_msgs::BoundingBoxArray :init))
	 (image_raw_msg))
    (setq image_raw_msg (one-shot-subscribe *head-camera-img*
					    sensor_msgs::Image
					    :after-stamp (ros::time-now)))
    (send *dimg-req-msg* :decos_img image_raw_msg)
    ))

(defun recognize-decoration ()
  (let* ((look_at_pos))
    (find-decoration (v- *look-at-pos* #f(0 0 100)))
    (close-decoration)
    (get-deco-image)
    ))

  