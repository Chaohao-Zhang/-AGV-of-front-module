;; Auto-generated. Do not edit!


(when (boundp 'move::detect_state)
  (if (not (find-package "MOVE"))
    (make-package "MOVE"))
  (shadow 'detect_state (find-package "MOVE")))
(unless (find-package "MOVE::DETECT_STATE")
  (make-package "MOVE::DETECT_STATE"))

(in-package "ROS")
;;//! \htmlinclude detect_state.msg.html


(defclass move::detect_state
  :super ros::object
  :slots (_detect_state ))

(defmethod move::detect_state
  (:init
   (&key
    ((:detect_state __detect_state) nil)
    )
   (send-super :init)
   (setq _detect_state __detect_state)
   self)
  (:detect_state
   (&optional __detect_state)
   (if __detect_state (setq _detect_state __detect_state)) _detect_state)
  (:serialization-length
   ()
   (+
    ;; bool _detect_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _detect_state
       (if _detect_state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _detect_state
     (setq _detect_state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get move::detect_state :md5sum-) "eb4ea15efeb62557142025112b232836")
(setf (get move::detect_state :datatype-) "move/detect_state")
(setf (get move::detect_state :definition-)
      "bool detect_state
")



(provide :move/detect_state "eb4ea15efeb62557142025112b232836")


