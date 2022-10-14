#!/usr/bin/env roseus

(setq *xyz-uv-req-topic* "/xyz_to_screen_point")
(setq *deco-req-topic* "/think_deco")

(defun debug-view-place-pos (i)
  (let* ((pos (elt *place-pos-lst* i))
	 (box (make-cube 50 50 50)))
    (send box :locate pos :world)
    (objects (list *fetch* box))
    ))

(defun convert-3D->2D (xyz_lst)
  ;; *base-frame* 3D pos -> *camera-frame* 2D pos
  (let* ((xyz_req_msg (instance deco_with_fetch::PointStampedRequest :init))
	 (x (elt xyz_lst 0)) (y (elt xyz_lst 1)) (z (elt xyz_lst 2))
	 (input_coords (make-coords :pos (float-vector x y z)))
	 (pos) (coords) (result_msg))
    (setq coords (send (send *tfl* :lookup-transform
			     *camera-frame* *base-frame* (ros::time 0))
		       :transform input_coords))
    (setq pos (send coords :pos))
    (send (send xyz_req_msg :point) :x (elt pos 0))
    (send (send xyz_req_msg :point) :y (elt pos 1))
    (send (send xyz_req_msg :point) :z (elt pos 2))
    (ros::wait-for-service *xyz-uv-req-topic*)
    (setq result_msg (ros::service-call *xyz-uv-req-topic* xyz_req_msg))
    result_msg)
  )

(defun get-corner-uv (pos dims)
  (let* ((lt_pos) (rt_pos) (lb_pos) (rb_pos)
	 (h_width (/ (elt dims 1) 2.0))
	 (h_length (/ (elt dims 0) 2.0))
	 (h_height (/ (elt dims 2) 2.0)))
    (setq lt_pos (v+ pos (float-vector h_length h_width h_height)))
    (setq rt_pos (v+ pos (float-vector h_length (* -1 h_width) h_height)))
    (setq lb_pos (v+ pos (float-vector (* -1 h_length) h_width h_height)))
    (setq rb_pos (v+ pos (float-vector (* -1 h_length) (* -1 h_width) h_height)))
    (setq lt_res (send (convert-3D->2D lt_pos) :point))
    (setq rt_res (send (convert-3D->2D rt_pos) :point))
    (setq lb_res (send (convert-3D->2D lb_pos) :point))
    (setq rb_res (send (convert-3D->2D rb_pos) :point))
    ;; (format t "xs: ~A~%" (list (send lt_res :x) (send rt_res :x) (send lb_res :x) (send rb_res :x)))
    ;; (format t "ys: ~A~%" (list (send lt_res :y) (send rt_res :y) (send lb_res :y) (send rb_res :y)))
    (list (list (send lt_res :x) (send rt_res :x) (send lb_res :x) (send rb_res :x))
	  (list (send lt_res :y) (send rt_res :y) (send lb_res :y) (send rb_res :y)))
    ))

(defun set-pub-msg ()
  (let* ((deco_box) (box_cd_lst) (pos) (dims) (pos_msg) (dims_msg)
	 (req_uv_msg) (xs_ys_lst))
    (send *dimg-req-msg* :header :stamp (ros::time-now))
    (dotimes (i (length *bboxes*))
      (setq deco_box (elt *bboxes* i))

      (setq box_cd_lst (bbox->lst deco_box))
      (setq pos (send (elt box_cd_lst 0) :pos))
      (setq dims (elt box_cd_lst 1))
      (format t "pos: ~A, dims ~A~%" pos dims)
      (setq pos_msg (instance geometry_msgs::Point :init))
      (setq dims_msg (instance geometry_msgs::Point :init))
      (send pos_msg :x (elt pos 0))
      (send pos_msg :y (elt pos 1))
      (send pos_msg :z (elt pos 2))
      (send dims_msg :x (elt dims 0))
      (send dims_msg :y (elt dims 1))
      (send dims_msg :z (elt dims 2))
      (send *dimg-req-msg* :decos_pos (append (send *dimg-req-msg* :decos_pos) (list pos_msg)))
      (send *dimg-req-msg* :decos_dims (append (send *dimg-req-msg* :decos_dims) (list dims_msg)))

      (setq rec_uv_msg (instance jsk_recognition_msgs::PlotData :init))
      (setq xs_ys_lst (get-corner-uv pos dims))
      (send rec_uv_msg :xs (elt xs_ys_lst 0))
      (send rec_uv_msg :ys (elt xs_ys_lst 1))
      (send *dimg-req-msg* :decos_rec_uv (append (send *dimg-req-msg* :decos_rec_uv) (list rec_uv_msg)))
      )
    ))

(defun store-result (result_pos)
  (let* ((x) (y) (result_arr)
	 (fetch_pose (send *ri* :state :potentio-vector)))
    (setq result_arr (send result_pos :poses))
    (setq pre_x -1)
    (make-reset-pose)  ;; to convert 2D->3D accurately
    (dotimes (i (length result_arr))
      (setq x (send (send (elt result_arr i) :position) :x))
      (setq y (send (send (elt result_arr i) :position) :y))
      ;; (format t "x: ~A, y: ~A~%" x y) 
      ;; use point_cloud of wall
      (convert-2D->3D x y *wall-pcl*)
      (setq *place-pos-lst* (append *place-pos-lst* (list result_3d_pos))))
    (format t "place_pos_lst: ~A~%" *place-pos-lst*)
    (send *fetch* :angle-vector fetch_pose)
    (send-robot 4000)
    ))

(defun think-decoration ()
  (let* ((result_msg))
    (set-pub-msg)
    (format t "waiting service ...~%")
    (ros::wait-for-service *deco-req-topic*)
    (setq result_msg (ros::service-call *deco-req-topic* *dimg-req-msg*))
    (format t "got service result~%")
    (store-result (send result_msg :result_pos))
  ))

