; Auto-generated. Do not edit!


(cl:in-package sunray_msgs-msg)


;//! \htmlinclude rmtt_cmd.msg.html

(cl:defclass <rmtt_cmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (agent_id
    :reader agent_id
    :initarg :agent_id
    :type cl:fixnum
    :initform 0)
   (control_state
    :reader control_state
    :initarg :control_state
    :type cl:fixnum
    :initform 0)
   (desired_pos
    :reader desired_pos
    :initarg :desired_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (desired_yaw
    :reader desired_yaw
    :initarg :desired_yaw
    :type cl:float
    :initform 0.0)
   (desired_vel
    :reader desired_vel
    :initarg :desired_vel
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass rmtt_cmd (<rmtt_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rmtt_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rmtt_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sunray_msgs-msg:<rmtt_cmd> is deprecated: use sunray_msgs-msg:rmtt_cmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <rmtt_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:header-val is deprecated.  Use sunray_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'agent_id-val :lambda-list '(m))
(cl:defmethod agent_id-val ((m <rmtt_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:agent_id-val is deprecated.  Use sunray_msgs-msg:agent_id instead.")
  (agent_id m))

(cl:ensure-generic-function 'control_state-val :lambda-list '(m))
(cl:defmethod control_state-val ((m <rmtt_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:control_state-val is deprecated.  Use sunray_msgs-msg:control_state instead.")
  (control_state m))

(cl:ensure-generic-function 'desired_pos-val :lambda-list '(m))
(cl:defmethod desired_pos-val ((m <rmtt_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:desired_pos-val is deprecated.  Use sunray_msgs-msg:desired_pos instead.")
  (desired_pos m))

(cl:ensure-generic-function 'desired_yaw-val :lambda-list '(m))
(cl:defmethod desired_yaw-val ((m <rmtt_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:desired_yaw-val is deprecated.  Use sunray_msgs-msg:desired_yaw instead.")
  (desired_yaw m))

(cl:ensure-generic-function 'desired_vel-val :lambda-list '(m))
(cl:defmethod desired_vel-val ((m <rmtt_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:desired_vel-val is deprecated.  Use sunray_msgs-msg:desired_vel instead.")
  (desired_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rmtt_cmd>) ostream)
  "Serializes a message object of type '<rmtt_cmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'agent_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_state)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_pos) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desired_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired_vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rmtt_cmd>) istream)
  "Deserializes a message object of type '<rmtt_cmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'agent_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_state)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_pos) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desired_yaw) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired_vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rmtt_cmd>)))
  "Returns string type for a message object of type '<rmtt_cmd>"
  "sunray_msgs/rmtt_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rmtt_cmd)))
  "Returns string type for a message object of type 'rmtt_cmd"
  "sunray_msgs/rmtt_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rmtt_cmd>)))
  "Returns md5sum for a message object of type '<rmtt_cmd>"
  "a1941fe70865cfa39c036a2c1ad0339a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rmtt_cmd)))
  "Returns md5sum for a message object of type 'rmtt_cmd"
  "a1941fe70865cfa39c036a2c1ad0339a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rmtt_cmd>)))
  "Returns full string definition for message of type '<rmtt_cmd>"
  (cl:format cl:nil "std_msgs/Header header~%~%## 当指定ID的时候，只有指定的ID响应~%uint8 agent_id  ~%~%## 状态机指令~%uint8 control_state   ~%~%## 期望位置、偏航角 -> 对应POS_CONTROL模式~%geometry_msgs/Point desired_pos    ## [m]~%float32 desired_yaw                 ## [rad]~%~%## 期望速度 -> 对应VEL_CONTROL模式~%geometry_msgs/Twist desired_vel~%~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rmtt_cmd)))
  "Returns full string definition for message of type 'rmtt_cmd"
  (cl:format cl:nil "std_msgs/Header header~%~%## 当指定ID的时候，只有指定的ID响应~%uint8 agent_id  ~%~%## 状态机指令~%uint8 control_state   ~%~%## 期望位置、偏航角 -> 对应POS_CONTROL模式~%geometry_msgs/Point desired_pos    ## [m]~%float32 desired_yaw                 ## [rad]~%~%## 期望速度 -> 对应VEL_CONTROL模式~%geometry_msgs/Twist desired_vel~%~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rmtt_cmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_pos))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired_vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rmtt_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'rmtt_cmd
    (cl:cons ':header (header msg))
    (cl:cons ':agent_id (agent_id msg))
    (cl:cons ':control_state (control_state msg))
    (cl:cons ':desired_pos (desired_pos msg))
    (cl:cons ':desired_yaw (desired_yaw msg))
    (cl:cons ':desired_vel (desired_vel msg))
))
