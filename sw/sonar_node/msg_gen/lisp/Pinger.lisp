; Auto-generated. Do not edit!


(cl:in-package sonar_node-msg)


;//! \htmlinclude Pinger.msg.html

(cl:defclass <Pinger> (roslisp-msg-protocol:ros-message)
  ((Id
    :reader Id
    :initarg :Id
    :type cl:fixnum
    :initform 0)
   (Heading
    :reader Heading
    :initarg :Heading
    :type cl:float
    :initform 0.0)
   (Magnitude
    :reader Magnitude
    :initarg :Magnitude
    :type cl:float
    :initform 0.0)
   (TimeSince
    :reader TimeSince
    :initarg :TimeSince
    :type cl:float
    :initform 0.0))
)

(cl:defclass Pinger (<Pinger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pinger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pinger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sonar_node-msg:<Pinger> is deprecated: use sonar_node-msg:Pinger instead.")))

(cl:ensure-generic-function 'Id-val :lambda-list '(m))
(cl:defmethod Id-val ((m <Pinger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar_node-msg:Id-val is deprecated.  Use sonar_node-msg:Id instead.")
  (Id m))

(cl:ensure-generic-function 'Heading-val :lambda-list '(m))
(cl:defmethod Heading-val ((m <Pinger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar_node-msg:Heading-val is deprecated.  Use sonar_node-msg:Heading instead.")
  (Heading m))

(cl:ensure-generic-function 'Magnitude-val :lambda-list '(m))
(cl:defmethod Magnitude-val ((m <Pinger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar_node-msg:Magnitude-val is deprecated.  Use sonar_node-msg:Magnitude instead.")
  (Magnitude m))

(cl:ensure-generic-function 'TimeSince-val :lambda-list '(m))
(cl:defmethod TimeSince-val ((m <Pinger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar_node-msg:TimeSince-val is deprecated.  Use sonar_node-msg:TimeSince instead.")
  (TimeSince m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pinger>) ostream)
  "Serializes a message object of type '<Pinger>"
  (cl:let* ((signed (cl:slot-value msg 'Id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Magnitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'TimeSince))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pinger>) istream)
  "Deserializes a message object of type '<Pinger>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Heading) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Magnitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'TimeSince) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pinger>)))
  "Returns string type for a message object of type '<Pinger>"
  "sonar_node/Pinger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pinger)))
  "Returns string type for a message object of type 'Pinger"
  "sonar_node/Pinger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pinger>)))
  "Returns md5sum for a message object of type '<Pinger>"
  "49d3e57ff39f4b02d41f58af18abc775")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pinger)))
  "Returns md5sum for a message object of type 'Pinger"
  "49d3e57ff39f4b02d41f58af18abc775")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pinger>)))
  "Returns full string definition for message of type '<Pinger>"
  (cl:format cl:nil "int8 Id           # 0 is correct pinger, 1 is incorrect pinger~%float64 Heading~%float64 Magnitude~%float64 TimeSince~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pinger)))
  "Returns full string definition for message of type 'Pinger"
  (cl:format cl:nil "int8 Id           # 0 is correct pinger, 1 is incorrect pinger~%float64 Heading~%float64 Magnitude~%float64 TimeSince~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pinger>))
  (cl:+ 0
     1
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pinger>))
  "Converts a ROS message object to a list"
  (cl:list 'Pinger
    (cl:cons ':Id (Id msg))
    (cl:cons ':Heading (Heading msg))
    (cl:cons ':Magnitude (Magnitude msg))
    (cl:cons ':TimeSince (TimeSince msg))
))
