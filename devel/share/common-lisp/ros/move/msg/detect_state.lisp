; Auto-generated. Do not edit!


(cl:in-package move-msg)


;//! \htmlinclude detect_state.msg.html

(cl:defclass <detect_state> (roslisp-msg-protocol:ros-message)
  ((detect_state
    :reader detect_state
    :initarg :detect_state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass detect_state (<detect_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detect_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detect_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move-msg:<detect_state> is deprecated: use move-msg:detect_state instead.")))

(cl:ensure-generic-function 'detect_state-val :lambda-list '(m))
(cl:defmethod detect_state-val ((m <detect_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move-msg:detect_state-val is deprecated.  Use move-msg:detect_state instead.")
  (detect_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detect_state>) ostream)
  "Serializes a message object of type '<detect_state>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'detect_state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detect_state>) istream)
  "Deserializes a message object of type '<detect_state>"
    (cl:setf (cl:slot-value msg 'detect_state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detect_state>)))
  "Returns string type for a message object of type '<detect_state>"
  "move/detect_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detect_state)))
  "Returns string type for a message object of type 'detect_state"
  "move/detect_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detect_state>)))
  "Returns md5sum for a message object of type '<detect_state>"
  "eb4ea15efeb62557142025112b232836")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detect_state)))
  "Returns md5sum for a message object of type 'detect_state"
  "eb4ea15efeb62557142025112b232836")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detect_state>)))
  "Returns full string definition for message of type '<detect_state>"
  (cl:format cl:nil "bool detect_state~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detect_state)))
  "Returns full string definition for message of type 'detect_state"
  (cl:format cl:nil "bool detect_state~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detect_state>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detect_state>))
  "Converts a ROS message object to a list"
  (cl:list 'detect_state
    (cl:cons ':detect_state (detect_state msg))
))
