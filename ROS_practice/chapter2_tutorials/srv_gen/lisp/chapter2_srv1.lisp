; Auto-generated. Do not edit!


(cl:in-package chapter2_tutorials-srv)


;//! \htmlinclude chapter2_srv1-request.msg.html

(cl:defclass <chapter2_srv1-request> (roslisp-msg-protocol:ros-message)
  ((A
    :reader A
    :initarg :A
    :type cl:integer
    :initform 0)
   (B
    :reader B
    :initarg :B
    :type cl:integer
    :initform 0)
   (C
    :reader C
    :initarg :C
    :type cl:integer
    :initform 0))
)

(cl:defclass chapter2_srv1-request (<chapter2_srv1-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chapter2_srv1-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chapter2_srv1-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chapter2_tutorials-srv:<chapter2_srv1-request> is deprecated: use chapter2_tutorials-srv:chapter2_srv1-request instead.")))

(cl:ensure-generic-function 'A-val :lambda-list '(m))
(cl:defmethod A-val ((m <chapter2_srv1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chapter2_tutorials-srv:A-val is deprecated.  Use chapter2_tutorials-srv:A instead.")
  (A m))

(cl:ensure-generic-function 'B-val :lambda-list '(m))
(cl:defmethod B-val ((m <chapter2_srv1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chapter2_tutorials-srv:B-val is deprecated.  Use chapter2_tutorials-srv:B instead.")
  (B m))

(cl:ensure-generic-function 'C-val :lambda-list '(m))
(cl:defmethod C-val ((m <chapter2_srv1-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chapter2_tutorials-srv:C-val is deprecated.  Use chapter2_tutorials-srv:C instead.")
  (C m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chapter2_srv1-request>) ostream)
  "Serializes a message object of type '<chapter2_srv1-request>"
  (cl:let* ((signed (cl:slot-value msg 'A)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'B)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'C)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chapter2_srv1-request>) istream)
  "Deserializes a message object of type '<chapter2_srv1-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'A) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'B) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'C) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chapter2_srv1-request>)))
  "Returns string type for a service object of type '<chapter2_srv1-request>"
  "chapter2_tutorials/chapter2_srv1Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chapter2_srv1-request)))
  "Returns string type for a service object of type 'chapter2_srv1-request"
  "chapter2_tutorials/chapter2_srv1Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chapter2_srv1-request>)))
  "Returns md5sum for a message object of type '<chapter2_srv1-request>"
  "2a5c7a37218262bae4fcfaa1007692aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chapter2_srv1-request)))
  "Returns md5sum for a message object of type 'chapter2_srv1-request"
  "2a5c7a37218262bae4fcfaa1007692aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chapter2_srv1-request>)))
  "Returns full string definition for message of type '<chapter2_srv1-request>"
  (cl:format cl:nil "int32 A~%int32 B~%int32 C~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chapter2_srv1-request)))
  "Returns full string definition for message of type 'chapter2_srv1-request"
  (cl:format cl:nil "int32 A~%int32 B~%int32 C~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chapter2_srv1-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chapter2_srv1-request>))
  "Converts a ROS message object to a list"
  (cl:list 'chapter2_srv1-request
    (cl:cons ':A (A msg))
    (cl:cons ':B (B msg))
    (cl:cons ':C (C msg))
))
;//! \htmlinclude chapter2_srv1-response.msg.html

(cl:defclass <chapter2_srv1-response> (roslisp-msg-protocol:ros-message)
  ((sum
    :reader sum
    :initarg :sum
    :type cl:integer
    :initform 0))
)

(cl:defclass chapter2_srv1-response (<chapter2_srv1-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chapter2_srv1-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chapter2_srv1-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chapter2_tutorials-srv:<chapter2_srv1-response> is deprecated: use chapter2_tutorials-srv:chapter2_srv1-response instead.")))

(cl:ensure-generic-function 'sum-val :lambda-list '(m))
(cl:defmethod sum-val ((m <chapter2_srv1-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chapter2_tutorials-srv:sum-val is deprecated.  Use chapter2_tutorials-srv:sum instead.")
  (sum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chapter2_srv1-response>) ostream)
  "Serializes a message object of type '<chapter2_srv1-response>"
  (cl:let* ((signed (cl:slot-value msg 'sum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chapter2_srv1-response>) istream)
  "Deserializes a message object of type '<chapter2_srv1-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sum) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chapter2_srv1-response>)))
  "Returns string type for a service object of type '<chapter2_srv1-response>"
  "chapter2_tutorials/chapter2_srv1Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chapter2_srv1-response)))
  "Returns string type for a service object of type 'chapter2_srv1-response"
  "chapter2_tutorials/chapter2_srv1Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chapter2_srv1-response>)))
  "Returns md5sum for a message object of type '<chapter2_srv1-response>"
  "2a5c7a37218262bae4fcfaa1007692aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chapter2_srv1-response)))
  "Returns md5sum for a message object of type 'chapter2_srv1-response"
  "2a5c7a37218262bae4fcfaa1007692aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chapter2_srv1-response>)))
  "Returns full string definition for message of type '<chapter2_srv1-response>"
  (cl:format cl:nil "int32 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chapter2_srv1-response)))
  "Returns full string definition for message of type 'chapter2_srv1-response"
  (cl:format cl:nil "int32 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chapter2_srv1-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chapter2_srv1-response>))
  "Converts a ROS message object to a list"
  (cl:list 'chapter2_srv1-response
    (cl:cons ':sum (sum msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'chapter2_srv1)))
  'chapter2_srv1-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'chapter2_srv1)))
  'chapter2_srv1-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chapter2_srv1)))
  "Returns string type for a service object of type '<chapter2_srv1>"
  "chapter2_tutorials/chapter2_srv1")