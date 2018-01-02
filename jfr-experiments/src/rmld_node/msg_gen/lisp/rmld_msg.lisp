; Auto-generated. Do not edit!


(cl:in-package rmld_node-msg)


;//! \htmlinclude rmld_msg.msg.html

(cl:defclass <rmld_msg> (roslisp-msg-protocol:ros-message)
  ((concentration_ppmm
    :reader concentration_ppmm
    :initarg :concentration_ppmm
    :type cl:float
    :initform 0.0)
   (rmld_data_string
    :reader rmld_data_string
    :initarg :rmld_data_string
    :type cl:string
    :initform ""))
)

(cl:defclass rmld_msg (<rmld_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rmld_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rmld_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rmld_node-msg:<rmld_msg> is deprecated: use rmld_node-msg:rmld_msg instead.")))

(cl:ensure-generic-function 'concentration_ppmm-val :lambda-list '(m))
(cl:defmethod concentration_ppmm-val ((m <rmld_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmld_node-msg:concentration_ppmm-val is deprecated.  Use rmld_node-msg:concentration_ppmm instead.")
  (concentration_ppmm m))

(cl:ensure-generic-function 'rmld_data_string-val :lambda-list '(m))
(cl:defmethod rmld_data_string-val ((m <rmld_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmld_node-msg:rmld_data_string-val is deprecated.  Use rmld_node-msg:rmld_data_string instead.")
  (rmld_data_string m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rmld_msg>) ostream)
  "Serializes a message object of type '<rmld_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'concentration_ppmm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'rmld_data_string))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'rmld_data_string))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rmld_msg>) istream)
  "Deserializes a message object of type '<rmld_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'concentration_ppmm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rmld_data_string) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'rmld_data_string) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rmld_msg>)))
  "Returns string type for a message object of type '<rmld_msg>"
  "rmld_node/rmld_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rmld_msg)))
  "Returns string type for a message object of type 'rmld_msg"
  "rmld_node/rmld_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rmld_msg>)))
  "Returns md5sum for a message object of type '<rmld_msg>"
  "1e11ef0acc09eeffee4c91b316534192")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rmld_msg)))
  "Returns md5sum for a message object of type 'rmld_msg"
  "1e11ef0acc09eeffee4c91b316534192")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rmld_msg>)))
  "Returns full string definition for message of type '<rmld_msg>"
  (cl:format cl:nil "float32 concentration_ppmm~%string rmld_data_string~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rmld_msg)))
  "Returns full string definition for message of type 'rmld_msg"
  (cl:format cl:nil "float32 concentration_ppmm~%string rmld_data_string~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rmld_msg>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'rmld_data_string))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rmld_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'rmld_msg
    (cl:cons ':concentration_ppmm (concentration_ppmm msg))
    (cl:cons ':rmld_data_string (rmld_data_string msg))
))
