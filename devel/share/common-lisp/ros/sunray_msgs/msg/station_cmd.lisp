; Auto-generated. Do not edit!


(cl:in-package sunray_msgs-msg)


;//! \htmlinclude station_cmd.msg.html

(cl:defclass <station_cmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (uav_id
    :reader uav_id
    :initarg :uav_id
    :type cl:fixnum
    :initform 0)
   (mission_state
    :reader mission_state
    :initarg :mission_state
    :type cl:fixnum
    :initform 0)
   (scenario_id
    :reader scenario_id
    :initarg :scenario_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass station_cmd (<station_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <station_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'station_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sunray_msgs-msg:<station_cmd> is deprecated: use sunray_msgs-msg:station_cmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <station_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:header-val is deprecated.  Use sunray_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'uav_id-val :lambda-list '(m))
(cl:defmethod uav_id-val ((m <station_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:uav_id-val is deprecated.  Use sunray_msgs-msg:uav_id instead.")
  (uav_id m))

(cl:ensure-generic-function 'mission_state-val :lambda-list '(m))
(cl:defmethod mission_state-val ((m <station_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:mission_state-val is deprecated.  Use sunray_msgs-msg:mission_state instead.")
  (mission_state m))

(cl:ensure-generic-function 'scenario_id-val :lambda-list '(m))
(cl:defmethod scenario_id-val ((m <station_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:scenario_id-val is deprecated.  Use sunray_msgs-msg:scenario_id instead.")
  (scenario_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <station_cmd>) ostream)
  "Serializes a message object of type '<station_cmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uav_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mission_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scenario_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <station_cmd>) istream)
  "Deserializes a message object of type '<station_cmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uav_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mission_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scenario_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<station_cmd>)))
  "Returns string type for a message object of type '<station_cmd>"
  "sunray_msgs/station_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'station_cmd)))
  "Returns string type for a message object of type 'station_cmd"
  "sunray_msgs/station_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<station_cmd>)))
  "Returns md5sum for a message object of type '<station_cmd>"
  "f9fbf99fef9ff42ede4e5c27986fb1df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'station_cmd)))
  "Returns md5sum for a message object of type 'station_cmd"
  "f9fbf99fef9ff42ede4e5c27986fb1df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<station_cmd>)))
  "Returns full string definition for message of type '<station_cmd>"
  (cl:format cl:nil "std_msgs/Header header~%~%## 对于起飞、降落、悬停、ORCA_RUN指令：当指定ID的时候，只有指定的ID响应~%## 对于其他指令，uav_id设置为99，所有飞机响应该状态  ~%uint8 uav_id  ~%~%## 状态机指令~%uint8 mission_state   ~%uint8 scenario_id           ~% ~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'station_cmd)))
  "Returns full string definition for message of type 'station_cmd"
  (cl:format cl:nil "std_msgs/Header header~%~%## 对于起飞、降落、悬停、ORCA_RUN指令：当指定ID的时候，只有指定的ID响应~%## 对于其他指令，uav_id设置为99，所有飞机响应该状态  ~%uint8 uav_id  ~%~%## 状态机指令~%uint8 mission_state   ~%uint8 scenario_id           ~% ~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <station_cmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <station_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'station_cmd
    (cl:cons ':header (header msg))
    (cl:cons ':uav_id (uav_id msg))
    (cl:cons ':mission_state (mission_state msg))
    (cl:cons ':scenario_id (scenario_id msg))
))
