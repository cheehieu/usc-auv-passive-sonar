; Auto-generated. Do not edit!


(cl:in-package sonar_node-srv)


;//! \htmlinclude FindPingers-request.msg.html

(cl:defclass <FindPingers-request> (roslisp-msg-protocol:ros-message)
  ((PingerIDs
    :reader PingerIDs
    :initarg :PingerIDs
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass FindPingers-request (<FindPingers-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindPingers-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindPingers-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sonar_node-srv:<FindPingers-request> is deprecated: use sonar_node-srv:FindPingers-request instead.")))

(cl:ensure-generic-function 'PingerIDs-val :lambda-list '(m))
(cl:defmethod PingerIDs-val ((m <FindPingers-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar_node-srv:PingerIDs-val is deprecated.  Use sonar_node-srv:PingerIDs instead.")
  (PingerIDs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindPingers-request>) ostream)
  "Serializes a message object of type '<FindPingers-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PingerIDs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'PingerIDs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindPingers-request>) istream)
  "Deserializes a message object of type '<FindPingers-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PingerIDs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PingerIDs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindPingers-request>)))
  "Returns string type for a service object of type '<FindPingers-request>"
  "sonar_node/FindPingersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindPingers-request)))
  "Returns string type for a service object of type 'FindPingers-request"
  "sonar_node/FindPingersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindPingers-request>)))
  "Returns md5sum for a message object of type '<FindPingers-request>"
  "3fc273c2cde0e4501b5b4f0338ae74b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindPingers-request)))
  "Returns md5sum for a message object of type 'FindPingers-request"
  "3fc273c2cde0e4501b5b4f0338ae74b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindPingers-request>)))
  "Returns full string definition for message of type '<FindPingers-request>"
  (cl:format cl:nil "uint8[] PingerIDs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindPingers-request)))
  "Returns full string definition for message of type 'FindPingers-request"
  (cl:format cl:nil "uint8[] PingerIDs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindPingers-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PingerIDs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindPingers-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FindPingers-request
    (cl:cons ':PingerIDs (PingerIDs msg))
))
;//! \htmlinclude FindPingers-response.msg.html

(cl:defclass <FindPingers-response> (roslisp-msg-protocol:ros-message)
  ((PingerArray
    :reader PingerArray
    :initarg :PingerArray
    :type sonar_node-msg:PingerArray
    :initform (cl:make-instance 'sonar_node-msg:PingerArray)))
)

(cl:defclass FindPingers-response (<FindPingers-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindPingers-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindPingers-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sonar_node-srv:<FindPingers-response> is deprecated: use sonar_node-srv:FindPingers-response instead.")))

(cl:ensure-generic-function 'PingerArray-val :lambda-list '(m))
(cl:defmethod PingerArray-val ((m <FindPingers-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar_node-srv:PingerArray-val is deprecated.  Use sonar_node-srv:PingerArray instead.")
  (PingerArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindPingers-response>) ostream)
  "Serializes a message object of type '<FindPingers-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'PingerArray) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindPingers-response>) istream)
  "Deserializes a message object of type '<FindPingers-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'PingerArray) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindPingers-response>)))
  "Returns string type for a service object of type '<FindPingers-response>"
  "sonar_node/FindPingersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindPingers-response)))
  "Returns string type for a service object of type 'FindPingers-response"
  "sonar_node/FindPingersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindPingers-response>)))
  "Returns md5sum for a message object of type '<FindPingers-response>"
  "3fc273c2cde0e4501b5b4f0338ae74b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindPingers-response)))
  "Returns md5sum for a message object of type 'FindPingers-response"
  "3fc273c2cde0e4501b5b4f0338ae74b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindPingers-response>)))
  "Returns full string definition for message of type '<FindPingers-response>"
  (cl:format cl:nil "sonar_node/PingerArray PingerArray~%~%~%================================================================================~%MSG: sonar_node/PingerArray~%sonar_node/Pinger[] Pingers~%~%================================================================================~%MSG: sonar_node/Pinger~%int8 Id           # 0 is correct pinger, 1 is incorrect pinger~%float64 Heading~%float64 Magnitude~%float64 TimeSince~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindPingers-response)))
  "Returns full string definition for message of type 'FindPingers-response"
  (cl:format cl:nil "sonar_node/PingerArray PingerArray~%~%~%================================================================================~%MSG: sonar_node/PingerArray~%sonar_node/Pinger[] Pingers~%~%================================================================================~%MSG: sonar_node/Pinger~%int8 Id           # 0 is correct pinger, 1 is incorrect pinger~%float64 Heading~%float64 Magnitude~%float64 TimeSince~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindPingers-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'PingerArray))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindPingers-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FindPingers-response
    (cl:cons ':PingerArray (PingerArray msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FindPingers)))
  'FindPingers-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FindPingers)))
  'FindPingers-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindPingers)))
  "Returns string type for a service object of type '<FindPingers>"
  "sonar_node/FindPingers")
