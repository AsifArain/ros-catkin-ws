; Auto-generated. Do not edit!


(cl:in-package ptu_control-srv)


;//! \htmlinclude commandSweep-request.msg.html

(cl:defclass <commandSweep-request> (roslisp-msg-protocol:ros-message)
  ((min_pan
    :reader min_pan
    :initarg :min_pan
    :type cl:float
    :initform 0.0)
   (max_pan
    :reader max_pan
    :initarg :max_pan
    :type cl:float
    :initform 0.0)
   (min_tilt
    :reader min_tilt
    :initarg :min_tilt
    :type cl:float
    :initform 0.0)
   (max_tilt
    :reader max_tilt
    :initarg :max_tilt
    :type cl:float
    :initform 0.0)
   (n_pan
    :reader n_pan
    :initarg :n_pan
    :type cl:fixnum
    :initform 0)
   (n_tilt
    :reader n_tilt
    :initarg :n_tilt
    :type cl:fixnum
    :initform 0)
   (samp_delay
    :reader samp_delay
    :initarg :samp_delay
    :type cl:float
    :initform 0.0))
)

(cl:defclass commandSweep-request (<commandSweep-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <commandSweep-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'commandSweep-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptu_control-srv:<commandSweep-request> is deprecated: use ptu_control-srv:commandSweep-request instead.")))

(cl:ensure-generic-function 'min_pan-val :lambda-list '(m))
(cl:defmethod min_pan-val ((m <commandSweep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptu_control-srv:min_pan-val is deprecated.  Use ptu_control-srv:min_pan instead.")
  (min_pan m))

(cl:ensure-generic-function 'max_pan-val :lambda-list '(m))
(cl:defmethod max_pan-val ((m <commandSweep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptu_control-srv:max_pan-val is deprecated.  Use ptu_control-srv:max_pan instead.")
  (max_pan m))

(cl:ensure-generic-function 'min_tilt-val :lambda-list '(m))
(cl:defmethod min_tilt-val ((m <commandSweep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptu_control-srv:min_tilt-val is deprecated.  Use ptu_control-srv:min_tilt instead.")
  (min_tilt m))

(cl:ensure-generic-function 'max_tilt-val :lambda-list '(m))
(cl:defmethod max_tilt-val ((m <commandSweep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptu_control-srv:max_tilt-val is deprecated.  Use ptu_control-srv:max_tilt instead.")
  (max_tilt m))

(cl:ensure-generic-function 'n_pan-val :lambda-list '(m))
(cl:defmethod n_pan-val ((m <commandSweep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptu_control-srv:n_pan-val is deprecated.  Use ptu_control-srv:n_pan instead.")
  (n_pan m))

(cl:ensure-generic-function 'n_tilt-val :lambda-list '(m))
(cl:defmethod n_tilt-val ((m <commandSweep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptu_control-srv:n_tilt-val is deprecated.  Use ptu_control-srv:n_tilt instead.")
  (n_tilt m))

(cl:ensure-generic-function 'samp_delay-val :lambda-list '(m))
(cl:defmethod samp_delay-val ((m <commandSweep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ptu_control-srv:samp_delay-val is deprecated.  Use ptu_control-srv:samp_delay instead.")
  (samp_delay m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <commandSweep-request>) ostream)
  "Serializes a message object of type '<commandSweep-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'min_tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'n_pan)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'n_tilt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'samp_delay))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <commandSweep-request>) istream)
  "Deserializes a message object of type '<commandSweep-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_pan) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_pan) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_tilt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_tilt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_pan) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_tilt) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'samp_delay) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<commandSweep-request>)))
  "Returns string type for a service object of type '<commandSweep-request>"
  "ptu_control/commandSweepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commandSweep-request)))
  "Returns string type for a service object of type 'commandSweep-request"
  "ptu_control/commandSweepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<commandSweep-request>)))
  "Returns md5sum for a message object of type '<commandSweep-request>"
  "d9bae297918c9ad58f81026a706beeea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'commandSweep-request)))
  "Returns md5sum for a message object of type 'commandSweep-request"
  "d9bae297918c9ad58f81026a706beeea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<commandSweep-request>)))
  "Returns full string definition for message of type '<commandSweep-request>"
  (cl:format cl:nil "float64 min_pan~%float64 max_pan~%float64 min_tilt~%float64 max_tilt~%int16 n_pan~%int16 n_tilt~%float64 samp_delay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'commandSweep-request)))
  "Returns full string definition for message of type 'commandSweep-request"
  (cl:format cl:nil "float64 min_pan~%float64 max_pan~%float64 min_tilt~%float64 max_tilt~%int16 n_pan~%int16 n_tilt~%float64 samp_delay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <commandSweep-request>))
  (cl:+ 0
     8
     8
     8
     8
     2
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <commandSweep-request>))
  "Converts a ROS message object to a list"
  (cl:list 'commandSweep-request
    (cl:cons ':min_pan (min_pan msg))
    (cl:cons ':max_pan (max_pan msg))
    (cl:cons ':min_tilt (min_tilt msg))
    (cl:cons ':max_tilt (max_tilt msg))
    (cl:cons ':n_pan (n_pan msg))
    (cl:cons ':n_tilt (n_tilt msg))
    (cl:cons ':samp_delay (samp_delay msg))
))
;//! \htmlinclude commandSweep-response.msg.html

(cl:defclass <commandSweep-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass commandSweep-response (<commandSweep-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <commandSweep-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'commandSweep-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ptu_control-srv:<commandSweep-response> is deprecated: use ptu_control-srv:commandSweep-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <commandSweep-response>) ostream)
  "Serializes a message object of type '<commandSweep-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <commandSweep-response>) istream)
  "Deserializes a message object of type '<commandSweep-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<commandSweep-response>)))
  "Returns string type for a service object of type '<commandSweep-response>"
  "ptu_control/commandSweepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commandSweep-response)))
  "Returns string type for a service object of type 'commandSweep-response"
  "ptu_control/commandSweepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<commandSweep-response>)))
  "Returns md5sum for a message object of type '<commandSweep-response>"
  "d9bae297918c9ad58f81026a706beeea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'commandSweep-response)))
  "Returns md5sum for a message object of type 'commandSweep-response"
  "d9bae297918c9ad58f81026a706beeea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<commandSweep-response>)))
  "Returns full string definition for message of type '<commandSweep-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'commandSweep-response)))
  "Returns full string definition for message of type 'commandSweep-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <commandSweep-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <commandSweep-response>))
  "Converts a ROS message object to a list"
  (cl:list 'commandSweep-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'commandSweep)))
  'commandSweep-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'commandSweep)))
  'commandSweep-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commandSweep)))
  "Returns string type for a service object of type '<commandSweep>"
  "ptu_control/commandSweep")