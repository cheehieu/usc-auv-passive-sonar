; Auto-generated. Do not edit!


(cl:in-package sonar_node-msg)


;//! \htmlinclude PingerArray.msg.html

(cl:defclass <PingerArray> (roslisp-msg-protocol:ros-message)
  ((Pingers
    :reader Pingers
    :initarg :Pingers
    :type (cl:vector sonar_node-msg:Pinger)
   :initform (cl:make-array 0 :element-type 'sonar_node-msg:Pinger :initial-element (cl:make-instance 'sonar_node-msg:Pinger))))
)

(cl:defclass PingerArray (<PingerArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PingerArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PingerArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sonar_node-msg:<PingerArray> is deprecated: use sonar_node-msg:PingerArray instead.")))

(cl:ensure-generic-function 'Pingers-val :lambda-list '(m))
(cl:defmethod Pingers-val ((m <PingerArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar_node-msg:Pingers-val is deprecated.  Use sonar_node-msg:Pingers instead.")
  (Pingers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PingerArray>) ostream)
  "Serializes a message object of type '<PingerArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Pingers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'Pingers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PingerArray>) istream)
  "Deserializes a message object of type '<PingerArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Pingers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Pingers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sonar_node-msg:Pinger))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PingerArray>)))
  "Returns string type for a message object of type '<PingerArray>"
  "sonar_node/PingerArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingerArray)))
  "Returns string type for a message object of type 'PingerArray"
  "sonar_node/PingerArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PingerArray>)))
  "Returns md5sum for a message object of type '<PingerArray>"
  "fd64935a04f55a3dfcff9fa3975c5353")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PingerArray)))
  "Returns md5sum for a message object of type 'PingerArray"
  "fd64935a04f55a3dfcff9fa3975c5353")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PingerArray>)))
  "Returns full string definition for message of type '<PingerArray>"
  (cl:format cl:nil "sonar_node/Pinger[] Pingers~%~%================================================================================~%MSG: sonar_node/Pinger~%int8 Id           # 0 is correct pinger, 1 is incorrect pinger~%float64 Heading~%float64 Magnitude~%float64 TimeSince~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PingerArray)))
  "Returns full string definition for message of type 'PingerArray"
  (cl:format cl:nil "sonar_node/Pinger[] Pingers~%~%================================================================================~%MSG: sonar_node/Pinger~%int8 Id           # 0 is correct pinger, 1 is incorrect pinger~%float64 Heading~%float64 Magnitude~%float64 TimeSince~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PingerArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Pingers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PingerArray>))
  "Converts a ROS message object to a list"
  (cl:list 'PingerArray
    (cl:cons ':Pingers (Pingers msg))
))
