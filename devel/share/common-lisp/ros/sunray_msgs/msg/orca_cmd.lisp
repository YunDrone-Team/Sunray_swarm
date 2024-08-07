; Auto-generated. Do not edit!


(cl:in-package sunray_msgs-msg)


;//! \htmlinclude orca_cmd.msg.html

(cl:defclass <orca_cmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (orca_cmd
    :reader orca_cmd
    :initarg :orca_cmd
    :type cl:fixnum
    :initform 0)
   (scenario_id
    :reader scenario_id
    :initarg :scenario_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass orca_cmd (<orca_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <orca_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'orca_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sunray_msgs-msg:<orca_cmd> is deprecated: use sunray_msgs-msg:orca_cmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <orca_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:header-val is deprecated.  Use sunray_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'orca_cmd-val :lambda-list '(m))
(cl:defmethod orca_cmd-val ((m <orca_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:orca_cmd-val is deprecated.  Use sunray_msgs-msg:orca_cmd instead.")
  (orca_cmd m))

(cl:ensure-generic-function 'scenario_id-val :lambda-list '(m))
(cl:defmethod scenario_id-val ((m <orca_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:scenario_id-val is deprecated.  Use sunray_msgs-msg:scenario_id instead.")
  (scenario_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <orca_cmd>) ostream)
  "Serializes a message object of type '<orca_cmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'orca_cmd)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scenario_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <orca_cmd>) istream)
  "Deserializes a message object of type '<orca_cmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'orca_cmd)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scenario_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<orca_cmd>)))
  "Returns string type for a message object of type '<orca_cmd>"
  "sunray_msgs/orca_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'orca_cmd)))
  "Returns string type for a message object of type 'orca_cmd"
  "sunray_msgs/orca_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<orca_cmd>)))
  "Returns md5sum for a message object of type '<orca_cmd>"
  "c1d519fdc68c652710d2bdf69a489838")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'orca_cmd)))
  "Returns md5sum for a message object of type 'orca_cmd"
  "c1d519fdc68c652710d2bdf69a489838")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<orca_cmd>)))
  "Returns full string definition for message of type '<orca_cmd>"
  (cl:format cl:nil "std_msgs/Header header~%~%## 状态机指令~%uint8 orca_cmd   ~%uint8 scenario_id           ~% ~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'orca_cmd)))
  "Returns full string definition for message of type 'orca_cmd"
  (cl:format cl:nil "std_msgs/Header header~%~%## 状态机指令~%uint8 orca_cmd   ~%uint8 scenario_id           ~% ~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <orca_cmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <orca_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'orca_cmd
    (cl:cons ':header (header msg))
    (cl:cons ':orca_cmd (orca_cmd msg))
    (cl:cons ':scenario_id (scenario_id msg))
))
