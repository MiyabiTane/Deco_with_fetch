#!/usr/bin/env roseus

(defun search-wall-range (u v)
  (let* ((u u) (v v))
    (convert-2D->3D u v *wall-pcl*)
    (while (or (= convert_flag 0)
	       (> (abs (- (elt result_3d_pos 0) *wall-x*)) 750))
      ;; (format t "3d_pos: ~A~%" result_3d_pos)
      ;; (format t "u: ~A, v: ~A~%" u v)
      (if (< u 320)
	  (setq u (+ u 20))
	(setq u (- u 20)))
       (convert-2D->3D u v *wall-pcl*))
    (list u v))
  )

(defun get-backimg-pos ()
  (let* ((point_msg (instance geometry_msgs::Point :init))
	 (uv_lst '()) (u) (v))
    (setq *wall-pcl*
	  (one-shot-subscribe *head-camera-points*
			      sensor_msgs::PointCloud2 :after-stamp (ros::time-now)))
    ;; set *wall-x*
    (convert-2D->3D 320 240 *wall-pcl*)
    (format t "center pos: ~A~%" result_3d_pos)
    (setq *wall-x* (elt result_3d_pos 0))

    (setq uv_lst (search-wall-range 0 0))
    (setq u (elt uv_lst 0))
    (setq v (elt uv_lst 1))
    (format t "left_top pos: ~A, ~A~%" u v)
    (setq point_msg (convert-2D->point-msg u v *wall-pcl*))
    (send *dimg-req-msg* :bimg_lt_pos point_msg)
    (setq point_msg (make-point-msg u v 0))
    (send *dimg-req-msg* :bimg_lt_uv point_msg)

    (setq uv_lst (search-wall-range 640 480))
    (setq u (elt uv_lst 0))
    (setq v (elt uv_lst 1))
    (format t "right_bottom pos: ~A, ~A~%" u v)
    (setq point_msg (convert-2D->point-msg u v *wall-pcl*))
    (send *dimg-req-msg* :bimg_rb_pos point_msg)
    (setq point_msg (make-point-msg u v 0))
    (send *dimg-req-msg* :bimg_rb_uv point_msg)
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


