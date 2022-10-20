#!/usr/bin/env roseus

(setq *status-req-topic* "/check_deco_status")

(defun get-decorated-uv ()
  (let* ((image_raw_msg
	  (one-shot-subscribe *head-camera-img*
			      sensor_msgs::Image :after-stamp (ros::time-now)))
	 (result_msg))
    (send *status-req-msg* :back_img image_raw_msg)
    (format t "deco-loop-num ~A, center-box-num ~A~%"
	    *deco-loop-num* *center-box-num*)
    (send *status-req-msg* :deco_count *deco-loop-num*)
    (send *status-req-msg* :center_box_num *center-box-num*)
    (format t "wating service ...~%")
    (ros::wait-for-service *status-req-topic*)
    (setq result_msg (ros::service-call *status-req-topic* *status-req-msg*))
    (format t "got service result~%")
    (format t "result ~A~%" (send result_msg :real_deco_uv))
    (send result_msg :real_deco_uv))
  )

(defun pointing-pos (target_pos)
  (let* ((target_pos target_pos)
	 (ref_vec (v- target_pos #f(50.4034 -127.561 787.276)))
	 (ref_vec (scale (/ 100.0 (norm ref_vec)) ref_vec))
	 (ik_res nil)
	 (offset_vec (scale
		      (/ (- (elt target_pos 0) 1100.0) (norm ref_vec)) ref_vec)))
    (setq target_pos (v- target_pos offset_vec))
    (format t "target_pos: ~A~%" target_pos)
    (while (and (not ik_res) (> (elt target_pos 0) 200))
      (setq ik_res (inv-from-reset-pose target_pos))
      (when (not ik_res)
	(setq target_pos (v- target_pos ref_vec))
	;; (view-debug-pos target_pos)
	))
    (send-robot 4000)
    ))

(defun check-deco-diff (result_lst)
  (let* ((result_lst result_lst)
	 (ideal_pos) (real_pos))
    (if (not result_lst)
	(progn
	  (send *ri* :speak-jp "飾り付けに失敗しました")
	  (format t "飾り付けに失敗しました~%"))
      (progn
	(dotimes (i (length *place-pos-lst*))
	  (when (not (= i *center-box-num*))
	    (when (not (= (send (elt result_lst i) :x) -1))
	      (setq ideal_pos (elt *place-pos-lst* i))
	      (convert-2D->3D (send (elt result_lst i) :x) (send (elt result_lst i) :y) *wall-pcl*) 
	      (setq real_pos result_3d_pos)
	      (format t "real_pos: ~A~%" real_pos)
	      (format t "ideal_pos: ~A~%" ideal_pos)
	      (pointing-pos real_pos) 
	      (format t "あの飾りは~%")
	      (ros::duration-sleep 3.0)
	      (pointing-pos ideal_pos)
	      (format t "あっちに置いてほしいです~%")
	      (ros::duration-sleep 3.0))
	    ))
	))
    ))

(defun check-decoration ()
  (let* ((result_lst))
    (setq result_lst (get-decorated-uv))
    (check-deco-diff result_lst)
    ))

(defun check-decoration-test ()
  (setq *place-pos-lst* (list #f(1906.57 344.389 1381.54) #f(2010.0 -728.395 766.317) #f(1895.04 464.742 481.844)))
  (setq *center-box-num* 1)
  (setq *wall-pcl*
	(one-shot-subscribe *head-camera-points*
			    sensor_msgs::PointCloud2 :after-stamp (ros::time-now)))
  (check-decoration)
  )
  
