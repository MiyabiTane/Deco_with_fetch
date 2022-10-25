#!/usr/bin/env roseus

(defun set-param-real ()
  (setq *CATCH-X-OFFSET* 0)
  (setq *CATCH-Z-OFFSET* 50)
  (setq *CATCH-BALLOON-TH* 0.05)  ;; 指の開き具合のずれ

  (setq *PLACE-POS-OFFSET* #f(1000 -200.6 0))
  (setq *PUSH-X-OFFSET* 0)  ;;400)  ;; 押す前に壁に近づく距離
  
  (setq *PUSH-TH-1* 0.2)  ;; 腕の位置のずれ
  (setq *PUSH-TH-2* 3.0)  ;; 5.0)  ;; 壁との距離
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
  (set-param-real)
  (init-global-param)
  (make-reset-pose)
  (send *ri* :stop-grasp)

  (rotate-ref-wall)
  (setq *wall-x* (get-distance-wall 320 240))

  (find-decoration (v- *look-at-pos* #f(0 0 100)) 5)
  (close-decoration)
  (set-catch-pos-lst)

  (setq cbox_num_pos (get-closest-box))
  (setq close_len_rot (before-catch (elt cbox_num_pos 0) (elt cbox_num_pos 1)))
  (catch-balloon-manip *CATCH-BALLOON-TH* 0)
  (after-catch (elt close_len_rot 0) (elt close_len_rot 1))
  ;; (go-vel-linear (/ -500 1000.0) 1)
  ;; (before-push-pose)
  ))

(defun place-balloon-real ()
  (set-param-real)
  (init-global-param)
  (make-reset-pose)
  (send *ri* :stop-grasp)

  ;; set goal_pos
  (setq *wall-x* (get-distance-wall 320 240))
  (setq place_pos (float-vector *wall-x* -700 1000))
  (format t "place_pos: ~A~%" place_pos)

  (set-before-catch-pose)
  (send-robot 4000)
  (send *ri* :speak-jp "風船を渡して下さい")
  (ros::duration-sleep 5.0)
  (catch-balloon-manip *CATCH-BALLOON-TH* 1)
  (before-push-pose)

  (setq close_len_rot (place-balloon-manip place_pos))
  (repush-balloon)
  (go-back (elt close_len_rot 0) (elt close_len_rot 1))
  ;; (go-vel-linear (/ -100 1000.0) 1)
  )

