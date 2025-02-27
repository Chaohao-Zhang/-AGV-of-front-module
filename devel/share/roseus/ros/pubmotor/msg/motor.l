;; Auto-generated. Do not edit!


(when (boundp 'pubmotor::motor)
  (if (not (find-package "PUBMOTOR"))
    (make-package "PUBMOTOR"))
  (shadow 'motor (find-package "PUBMOTOR")))
(unless (find-package "PUBMOTOR::MOTOR")
  (make-package "PUBMOTOR::MOTOR"))

(in-package "ROS")
;;//! \htmlinclude motor.msg.html


(defclass pubmotor::motor
  :super ros::object
  :slots (_id _velocity _acceleration _position ))

(defmethod pubmotor::motor
  (:init
   (&key
    ((:id __id) "")
    ((:velocity __velocity) 0.0)
    ((:acceleration __acceleration) 0.0)
    ((:position __position) 0.0)
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _velocity (float __velocity))
   (setq _acceleration (float __acceleration))
   (setq _position (float __position))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; float32 _velocity
    4
    ;; float32 _acceleration
    4
    ;; float32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _position
       (sys::poke _position (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _position
     (setq _position (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pubmotor::motor :md5sum-) "db2e79cc54e53959364c0e43a9d2ea58")
(setf (get pubmotor::motor :datatype-) "pubmotor/motor")
(setf (get pubmotor::motor :definition-)
      "string id
float32 velocity 
float32 acceleration
float32 position
")



(provide :pubmotor/motor "db2e79cc54e53959364c0e43a9d2ea58")


