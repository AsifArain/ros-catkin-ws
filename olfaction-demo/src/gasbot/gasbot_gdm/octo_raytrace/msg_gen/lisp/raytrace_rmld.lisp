; Auto-generated. Do not edit!


(cl:in-package octo_raytrace-msg)


;//! \htmlinclude raytrace_rmld.msg.html

(cl:defclass <raytrace_rmld> (roslisp-msg-protocol:ros-message)
  ((startX
    :reader startX
    :initarg :startX
    :type cl:float
    :initform 0.0)
   (startY
    :reader startY
    :initarg :startY
    :type cl:float
    :initform 0.0)
   (startZ
    :reader startZ
    :initarg :startZ
    :type cl:float
    :initform 0.0)
   (endX
    :reader endX
    :initarg :endX
    :type cl:float
    :initform 0.0)
   (endY
    :reader endY
    :initarg :endY
    :type cl:float
    :initform 0.0)
   (endZ
    :reader endZ
    :initarg :endZ
    :type cl:float
    :initform 0.0)
   (ppmm
    :reader ppmm
    :initarg :ppmm
    :type cl:float
    :initform 0.0))
)

(cl:defclass raytrace_rmld (<raytrace_rmld>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <raytrace_rmld>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'raytrace_rmld)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name octo_raytrace-msg:<raytrace_rmld> is deprecated: use octo_raytrace-msg:raytrace_rmld instead.")))

(cl:ensure-generic-function 'startX-val :lambda-list '(m))
(cl:defmethod startX-val ((m <raytrace_rmld>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader octo_raytrace-msg:startX-val is deprecated.  Use octo_raytrace-msg:startX instead.")
  (startX m))

(cl:ensure-generic-function 'startY-val :lambda-list '(m))
(cl:defmethod startY-val ((m <raytrace_rmld>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader octo_raytrace-msg:startY-val is deprecated.  Use octo_raytrace-msg:startY instead.")
  (startY m))

(cl:ensure-generic-function 'startZ-val :lambda-list '(m))
(cl:defmethod startZ-val ((m <raytrace_rmld>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader octo_raytrace-msg:startZ-val is deprecated.  Use octo_raytrace-msg:startZ instead.")
  (startZ m))

(cl:ensure-generic-function 'endX-val :lambda-list '(m))
(cl:defmethod endX-val ((m <raytrace_rmld>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader octo_raytrace-msg:endX-val is deprecated.  Use octo_raytrace-msg:endX instead.")
  (endX m))

(cl:ensure-generic-function 'endY-val :lambda-list '(m))
(cl:defmethod endY-val ((m <raytrace_rmld>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader octo_raytrace-msg:endY-val is deprecated.  Use octo_raytrace-msg:endY instead.")
  (endY m))

(cl:ensure-generic-function 'endZ-val :lambda-list '(m))
(cl:defmethod endZ-val ((m <raytrace_rmld>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader octo_raytrace-msg:endZ-val is deprecated.  Use octo_raytrace-msg:endZ instead.")
  (endZ m))

(cl:ensure-generic-function 'ppmm-val :lambda-list '(m))
(cl:defmethod ppmm-val ((m <raytrace_rmld>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader octo_raytrace-msg:ppmm-val is deprecated.  Use octo_raytrace-msg:ppmm instead.")
  (ppmm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <raytrace_rmld>) ostream)
  "Serializes a message object of type '<raytrace_rmld>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'startX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'startY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'startZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'endX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'endY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'endZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ppmm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <raytrace_rmld>) istream)
  "Deserializes a message object of type '<raytrace_rmld>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'startX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'startY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'startZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'endX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'endY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'endZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ppmm) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<raytrace_rmld>)))
  "Returns string type for a message object of type '<raytrace_rmld>"
  "octo_raytrace/raytrace_rmld")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'raytrace_rmld)))
  "Returns string type for a message object of type 'raytrace_rmld"
  "octo_raytrace/raytrace_rmld")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<raytrace_rmld>)))
  "Returns md5sum for a message object of type '<raytrace_rmld>"
  "fc2bde0ec68eb095a15b4032843ae8dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'raytrace_rmld)))
  "Returns md5sum for a message object of type 'raytrace_rmld"
  "fc2bde0ec68eb095a15b4032843ae8dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<raytrace_rmld>)))
  "Returns full string definition for message of type '<raytrace_rmld>"
  (cl:format cl:nil "float32 startX~%float32 startY~%float32 startZ~%float32 endX~%float32 endY~%float32 endZ~%float32 ppmm~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'raytrace_rmld)))
  "Returns full string definition for message of type 'raytrace_rmld"
  (cl:format cl:nil "float32 startX~%float32 startY~%float32 startZ~%float32 endX~%float32 endY~%float32 endZ~%float32 ppmm~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <raytrace_rmld>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <raytrace_rmld>))
  "Converts a ROS message object to a list"
  (cl:list 'raytrace_rmld
    (cl:cons ':startX (startX msg))
    (cl:cons ':startY (startY msg))
    (cl:cons ':startZ (startZ msg))
    (cl:cons ':endX (endX msg))
    (cl:cons ':endY (endY msg))
    (cl:cons ':endZ (endZ msg))
    (cl:cons ':ppmm (ppmm msg))
))
