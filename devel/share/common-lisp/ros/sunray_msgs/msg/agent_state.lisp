; Auto-generated. Do not edit!


(cl:in-package sunray_msgs-msg)


;//! \htmlinclude agent_state.msg.html

(cl:defclass <agent_state> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (agent_type
    :reader agent_type
    :initarg :agent_type
    :type cl:fixnum
    :initform 0)
   (agent_id
    :reader agent_id
    :initarg :agent_id
    :type cl:fixnum
    :initform 0)
   (agent_ip
    :reader agent_ip
    :initarg :agent_ip
    :type cl:string
    :initform "")
   (connected
    :reader connected
    :initarg :connected
    :type cl:boolean
    :initform cl:nil)
   (odom_valid
    :reader odom_valid
    :initarg :odom_valid
    :type cl:boolean
    :initform cl:nil)
   (pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (vel
    :reader vel
    :initarg :vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (att
    :reader att
    :initarg :att
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (attitude_q
    :reader attitude_q
    :initarg :attitude_q
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (battery
    :reader battery
    :initarg :battery
    :type cl:float
    :initform 0.0)
   (control_state
    :reader control_state
    :initarg :control_state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass agent_state (<agent_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <agent_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'agent_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sunray_msgs-msg:<agent_state> is deprecated: use sunray_msgs-msg:agent_state instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:header-val is deprecated.  Use sunray_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'agent_type-val :lambda-list '(m))
(cl:defmethod agent_type-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:agent_type-val is deprecated.  Use sunray_msgs-msg:agent_type instead.")
  (agent_type m))

(cl:ensure-generic-function 'agent_id-val :lambda-list '(m))
(cl:defmethod agent_id-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:agent_id-val is deprecated.  Use sunray_msgs-msg:agent_id instead.")
  (agent_id m))

(cl:ensure-generic-function 'agent_ip-val :lambda-list '(m))
(cl:defmethod agent_ip-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:agent_ip-val is deprecated.  Use sunray_msgs-msg:agent_ip instead.")
  (agent_ip m))

(cl:ensure-generic-function 'connected-val :lambda-list '(m))
(cl:defmethod connected-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:connected-val is deprecated.  Use sunray_msgs-msg:connected instead.")
  (connected m))

(cl:ensure-generic-function 'odom_valid-val :lambda-list '(m))
(cl:defmethod odom_valid-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:odom_valid-val is deprecated.  Use sunray_msgs-msg:odom_valid instead.")
  (odom_valid m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:pos-val is deprecated.  Use sunray_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:vel-val is deprecated.  Use sunray_msgs-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'att-val :lambda-list '(m))
(cl:defmethod att-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:att-val is deprecated.  Use sunray_msgs-msg:att instead.")
  (att m))

(cl:ensure-generic-function 'attitude_q-val :lambda-list '(m))
(cl:defmethod attitude_q-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:attitude_q-val is deprecated.  Use sunray_msgs-msg:attitude_q instead.")
  (attitude_q m))

(cl:ensure-generic-function 'battery-val :lambda-list '(m))
(cl:defmethod battery-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:battery-val is deprecated.  Use sunray_msgs-msg:battery instead.")
  (battery m))

(cl:ensure-generic-function 'control_state-val :lambda-list '(m))
(cl:defmethod control_state-val ((m <agent_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:control_state-val is deprecated.  Use sunray_msgs-msg:control_state instead.")
  (control_state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<agent_state>)))
    "Constants for message type '<agent_state>"
  '((:RMTT . 0)
    (:TIANBOT . 1)
    (:WHEELTEC . 2)
    (:SIKONG . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'agent_state)))
    "Constants for message type 'agent_state"
  '((:RMTT . 0)
    (:TIANBOT . 1)
    (:WHEELTEC . 2)
    (:SIKONG . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <agent_state>) ostream)
  "Serializes a message object of type '<agent_state>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'agent_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'agent_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'agent_ip))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'agent_ip))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'connected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'odom_valid) 1 0)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pos))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'vel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'att))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_q) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <agent_state>) istream)
  "Deserializes a message object of type '<agent_state>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'agent_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'agent_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'agent_ip) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'agent_ip) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'connected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'odom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'pos)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'vel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'vel)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'att) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'att)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_q) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<agent_state>)))
  "Returns string type for a message object of type '<agent_state>"
  "sunray_msgs/agent_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agent_state)))
  "Returns string type for a message object of type 'agent_state"
  "sunray_msgs/agent_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<agent_state>)))
  "Returns md5sum for a message object of type '<agent_state>"
  "adb46b835ede27744c5ee8dbd1eaf160")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'agent_state)))
  "Returns md5sum for a message object of type 'agent_state"
  "adb46b835ede27744c5ee8dbd1eaf160")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<agent_state>)))
  "Returns full string definition for message of type '<agent_state>"
  (cl:format cl:nil "std_msgs/Header header~%~%## 基本状态~%uint8 agent_type              ## 智能体类型~%uint8 agent_id                ## 智能体编号~%string agent_ip                 ## 智能体IP~%bool connected                ## 是否连接上智能体驱动~%bool odom_valid               ## 是否收到动捕数据~%~%## 智能体位置、速度、姿态~%float32[3] pos                 ## [m]~%float32[3] vel                 ## [m/s]~%float32[3] att                 ## [rad]~%geometry_msgs/Quaternion attitude_q ## 四元数~%~%## 智能体电池状态~%float32 battery                ## [0-1]~%~%## 智能体控制状态~%uint8 control_state~%~%## agent_type枚举~%uint8 RMTT = 0~%uint8 TIANBOT = 1~%uint8 WHEELTEC = 2~%uint8 SIKONG = 3~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'agent_state)))
  "Returns full string definition for message of type 'agent_state"
  (cl:format cl:nil "std_msgs/Header header~%~%## 基本状态~%uint8 agent_type              ## 智能体类型~%uint8 agent_id                ## 智能体编号~%string agent_ip                 ## 智能体IP~%bool connected                ## 是否连接上智能体驱动~%bool odom_valid               ## 是否收到动捕数据~%~%## 智能体位置、速度、姿态~%float32[3] pos                 ## [m]~%float32[3] vel                 ## [m/s]~%float32[3] att                 ## [rad]~%geometry_msgs/Quaternion attitude_q ## 四元数~%~%## 智能体电池状态~%float32 battery                ## [0-1]~%~%## 智能体控制状态~%uint8 control_state~%~%## agent_type枚举~%uint8 RMTT = 0~%uint8 TIANBOT = 1~%uint8 WHEELTEC = 2~%uint8 SIKONG = 3~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <agent_state>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4 (cl:length (cl:slot-value msg 'agent_ip))
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'att) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_q))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <agent_state>))
  "Converts a ROS message object to a list"
  (cl:list 'agent_state
    (cl:cons ':header (header msg))
    (cl:cons ':agent_type (agent_type msg))
    (cl:cons ':agent_id (agent_id msg))
    (cl:cons ':agent_ip (agent_ip msg))
    (cl:cons ':connected (connected msg))
    (cl:cons ':odom_valid (odom_valid msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':att (att msg))
    (cl:cons ':attitude_q (attitude_q msg))
    (cl:cons ':battery (battery msg))
    (cl:cons ':control_state (control_state msg))
))
