; Auto-generated. Do not edit!


(cl:in-package ros_service_assignment-srv)


;//! \htmlinclude RectangleAreaService-request.msg.html

(cl:defclass <RectangleAreaService-request> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0))
)

(cl:defclass RectangleAreaService-request (<RectangleAreaService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RectangleAreaService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RectangleAreaService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_service_assignment-srv:<RectangleAreaService-request> is deprecated: use ros_service_assignment-srv:RectangleAreaService-request instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <RectangleAreaService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_service_assignment-srv:width-val is deprecated.  Use ros_service_assignment-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <RectangleAreaService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_service_assignment-srv:height-val is deprecated.  Use ros_service_assignment-srv:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RectangleAreaService-request>) ostream)
  "Serializes a message object of type '<RectangleAreaService-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RectangleAreaService-request>) istream)
  "Deserializes a message object of type '<RectangleAreaService-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RectangleAreaService-request>)))
  "Returns string type for a service object of type '<RectangleAreaService-request>"
  "ros_service_assignment/RectangleAreaServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectangleAreaService-request)))
  "Returns string type for a service object of type 'RectangleAreaService-request"
  "ros_service_assignment/RectangleAreaServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RectangleAreaService-request>)))
  "Returns md5sum for a message object of type '<RectangleAreaService-request>"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RectangleAreaService-request)))
  "Returns md5sum for a message object of type 'RectangleAreaService-request"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RectangleAreaService-request>)))
  "Returns full string definition for message of type '<RectangleAreaService-request>"
  (cl:format cl:nil "float32 width~%float32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RectangleAreaService-request)))
  "Returns full string definition for message of type 'RectangleAreaService-request"
  (cl:format cl:nil "float32 width~%float32 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RectangleAreaService-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RectangleAreaService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RectangleAreaService-request
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
;//! \htmlinclude RectangleAreaService-response.msg.html

(cl:defclass <RectangleAreaService-response> (roslisp-msg-protocol:ros-message)
  ((area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0))
)

(cl:defclass RectangleAreaService-response (<RectangleAreaService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RectangleAreaService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RectangleAreaService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_service_assignment-srv:<RectangleAreaService-response> is deprecated: use ros_service_assignment-srv:RectangleAreaService-response instead.")))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <RectangleAreaService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_service_assignment-srv:area-val is deprecated.  Use ros_service_assignment-srv:area instead.")
  (area m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RectangleAreaService-response>) ostream)
  "Serializes a message object of type '<RectangleAreaService-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RectangleAreaService-response>) istream)
  "Deserializes a message object of type '<RectangleAreaService-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RectangleAreaService-response>)))
  "Returns string type for a service object of type '<RectangleAreaService-response>"
  "ros_service_assignment/RectangleAreaServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectangleAreaService-response)))
  "Returns string type for a service object of type 'RectangleAreaService-response"
  "ros_service_assignment/RectangleAreaServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RectangleAreaService-response>)))
  "Returns md5sum for a message object of type '<RectangleAreaService-response>"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RectangleAreaService-response)))
  "Returns md5sum for a message object of type 'RectangleAreaService-response"
  "92e9c8f940da77dc3e1bc289f7dd146e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RectangleAreaService-response>)))
  "Returns full string definition for message of type '<RectangleAreaService-response>"
  (cl:format cl:nil "float32 area~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RectangleAreaService-response)))
  "Returns full string definition for message of type 'RectangleAreaService-response"
  (cl:format cl:nil "float32 area~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RectangleAreaService-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RectangleAreaService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RectangleAreaService-response
    (cl:cons ':area (area msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RectangleAreaService)))
  'RectangleAreaService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RectangleAreaService)))
  'RectangleAreaService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RectangleAreaService)))
  "Returns string type for a service object of type '<RectangleAreaService>"
  "ros_service_assignment/RectangleAreaService")