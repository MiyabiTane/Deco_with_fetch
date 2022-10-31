#!/usr/bin/env roseus

(defun get-backimg-pos ()
  (let* ((point_msg (instance geometry_msgs::Point :init)))
    (setq *wall-pcl*
	  (one-shot-subscribe *head-camera-points*
			      sensor_msgs::PointCloud2 :after-stamp (ros::time-now)))
    (format t "left_top pos: ")
    (setq point_msg (convert-2D->point-msg 240 180 *wall-pcl*))
    (send *dimg-req-msg* :bimg_lt_pos point_msg)
    (setq point_msg (make-point-msg 240 0 0))
    (send *dimg-req-msg* :bimg_lt_uv point_msg)
    (format t "right_bottom pos: ")
    (setq point_msg (convert-2D->point-msg 400 300 *wall-pcl*))
    (send *dimg-req-msg* :bimg_rb_pos point_msg)
    (setq point_msg (make-point-msg 400 480 0))
    (send *dimg-req-msg* :bimg_rb_uv point_msg)
    ;; set *wall-x*
    (convert-2D->3D 320 240 *wall-pcl*)
    (format t "center pos: ~A~%" result_3d_pos)
    (setq *wall-x* (elt result_3d_pos 0))
    ))

(defun recognize-wall ()
  (let* ((image_raw_msg
	  (one-shot-subscribe *head-camera-img*
			      sensor_msgs::Image :after-stamp (ros::time-now))))
    (rotate-ref-wall)
    (format t "finish rotate~%")
    (send *dimg-req-msg* :back_img image_raw_msg)
    (get-backimg-pos)
    ))


