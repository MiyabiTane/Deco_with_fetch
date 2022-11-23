#!/usr/bin/env roseus

(defun set-param-real ()
  (setq *CATCH-BALLOON-TH* 0.05)  ;; 指の開き具合のずれ

  ;; (setq *PLACE-POS-OFFSET* #f(1000 -200.6 0))
  (setq *PLACE-POS-OFFSET* #f(1000.0 -400.0 0.0))
  ;; (setq *PUSH-X-OFFSET* 0)  ;;400)  ;; 押す前に壁に近づく距離
  
  (setq *PUSH-TH-1* 0.2)  ;; 腕の位置のずれ
  (setq *PUSH-TH-2* 5.0)  ;; 5.0)  ;; 壁との距離
  )

(defun set-catch-pos-lst ()
  (let* ((deco_box) (box_cd_lst) (pos))
    (dotimes (i (length *bboxes*))
      (setq deco_box (elt *bboxes* i))
      (setq box_cd_lst (bbox->lst deco_box))
      (setq pos (send (elt box_cd_lst 0) :pos))
      (format t "pos: ~A~%" pos)
      (setq *catch-pos-lst* (append *catch-pos-lst* (list pos)))
      )
    ))

(defun catch-balloon-real ()
  (set-param-real)
  (init-global-param)
  (make-reset-pose)
  (send *ri* :stop-grasp)

  (set-before-catch-pose)
  (send-robot 4000)
  (send *ri* :speak-jp "風船を渡して下さい")
  (ros::duration-sleep 5.0)
  (catch-balloon-manip *CATCH-BALLOON-TH* 0)
  (before-push-pose)
  )

(defun recog-and-catch-real ()

  (defun after-catch ()
    (let* ((wall_x) (rot_num)
	   (leave_x (* -1 *close-deco-x*)))
      (set-before-catch-pose)
      (send-robot 4000)
      (before-push-pose)
      (format t "leave_x: ~A~%" leave_x)
      (go-vel-linear (/ leave_x 1000.0) 1)
      (setq rot_num (- (floor (/ *rotate-rad* (/ pi 4))) 1))
      (format t "rot_num: ~A~%" rot_num)
      (dotimes (i rot_num)
	(go-vel-rotate (* -1 (/ pi 4))))
      (ros::duration-sleep 1.0)
      ))

  (set-param-real)
  (init-global-param)
  (make-reset-pose)
  (send *ri* :stop-grasp)

  (find-decoration (v- *look-at-pos* #f(0 0 100)) 5)
  (close-decoration)
  (set-catch-pos-lst)

  (setq cbox_num_pos (get-closest-box))
  (before-catch (elt cbox_num_pos 1))
  (catch-balloon-manip *CATCH-BALLOON-TH* 0)
  (after-catch)
  ))

(defun place-balloon-real ()
  ;; (set-param-real)
  (init-global-param)
  (make-reset-pose)
  (send *ri* :stop-grasp)

  ;; set goal_pos
  (setq *wall-x* (get-distance-wall 320 240))
  (setq place_pos (float-vector *wall-x* -200 1000))
  (format t "place_pos: ~A~%" place_pos)

  (rotate-ref-wall 0.0 (deg2rad 15))

  (set-before-catch-pose)
  (send-robot 4000)
  (send *ri* :speak-jp "風船を渡して下さい")
  (ros::duration-sleep 5.0)
  (catch-balloon-manip *CATCH-BALLOON-TH* 0)
  (before-push-pose)

  (setq close_len_rot (place-balloon-manip place_pos))
  (if (not *pass-to-human-flag*)
      (repush-balloon))
  (go-back (elt close_len_rot 0) (elt close_len_rot 1) (elt close_len_rot 2) (elt close_len_rot 3))
  )

(defun push-balloon-real-wall ()
  (init-global-param)
  (before-push-pose)
  (catch-balloon-manip 0.05 1)
  (before-push-pose)
  (push-balloon 0.2 5.0 20 20)
  (send *ri* :stop-grasp)
  (send *fetch* :rarm :move-end-pos #f(-20 0 0) :world)
  (send-robot 2000)
  (send *fetch* :rarm :move-end-pos #f(0 0 -150) :world)
  (send-robot 2000)
  (send *fetch* :rarm :move-end-pos #f(-150 0 0) :world)
  (send-robot 2000)
  (repush-balloon)
  )
