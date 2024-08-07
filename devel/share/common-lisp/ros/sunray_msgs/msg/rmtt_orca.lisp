; Auto-generated. Do not edit!


(cl:in-package sunray_msgs-msg)


;//! \htmlinclude rmtt_orca.msg.html

(cl:defclass <rmtt_orca> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mission_state
    :reader mission_state
    :initarg :mission_state
    :type cl:fixnum
    :initform 0)
   (uav_id
    :reader uav_id
    :initarg :uav_id
    :type cl:fixnum
    :initform 0)
   (arrived_goal
    :reader arrived_goal
    :initarg :arrived_goal
    :type cl:boolean
    :initform cl:nil)
   (arrived_all_goal
    :reader arrived_all_goal
    :initarg :arrived_all_goal
    :type cl:boolean
    :initform cl:nil)
   (goal
    :reader goal
    :initarg :goal
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (vel_orca
    :reader vel_orca
    :initarg :vel_orca
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass rmtt_orca (<rmtt_orca>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rmtt_orca>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rmtt_orca)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sunray_msgs-msg:<rmtt_orca> is deprecated: use sunray_msgs-msg:rmtt_orca instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <rmtt_orca>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:header-val is deprecated.  Use sunray_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mission_state-val :lambda-list '(m))
(cl:defmethod mission_state-val ((m <rmtt_orca>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:mission_state-val is deprecated.  Use sunray_msgs-msg:mission_state instead.")
  (mission_state m))

(cl:ensure-generic-function 'uav_id-val :lambda-list '(m))
(cl:defmethod uav_id-val ((m <rmtt_orca>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:uav_id-val is deprecated.  Use sunray_msgs-msg:uav_id instead.")
  (uav_id m))

(cl:ensure-generic-function 'arrived_goal-val :lambda-list '(m))
(cl:defmethod arrived_goal-val ((m <rmtt_orca>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:arrived_goal-val is deprecated.  Use sunray_msgs-msg:arrived_goal instead.")
  (arrived_goal m))

(cl:ensure-generic-function 'arrived_all_goal-val :lambda-list '(m))
(cl:defmethod arrived_all_goal-val ((m <rmtt_orca>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:arrived_all_goal-val is deprecated.  Use sunray_msgs-msg:arrived_all_goal instead.")
  (arrived_all_goal m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <rmtt_orca>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:goal-val is deprecated.  Use sunray_msgs-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'vel_orca-val :lambda-list '(m))
(cl:defmethod vel_orca-val ((m <rmtt_orca>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sunray_msgs-msg:vel_orca-val is deprecated.  Use sunray_msgs-msg:vel_orca instead.")
  (vel_orca m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rmtt_orca>) ostream)
  "Serializes a message object of type '<rmtt_orca>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mission_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uav_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arrived_goal) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arrived_all_goal) 1 0)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'goal))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'vel_orca))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rmtt_orca>) istream)
  "Deserializes a message object of type '<rmtt_orca>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mission_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uav_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arrived_goal) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'arrived_all_goal) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'goal) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'goal)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'vel_orca) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'vel_orca)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rmtt_orca>)))
  "Returns string type for a message object of type '<rmtt_orca>"
  "sunray_msgs/rmtt_orca")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rmtt_orca)))
  "Returns string type for a message object of type 'rmtt_orca"
  "sunray_msgs/rmtt_orca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rmtt_orca>)))
  "Returns md5sum for a message object of type '<rmtt_orca>"
  "94cd23020ece232971ee6a1b721a7e0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rmtt_orca)))
  "Returns md5sum for a message object of type 'rmtt_orca"
  "94cd23020ece232971ee6a1b721a7e0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rmtt_orca>)))
  "Returns full string definition for message of type '<rmtt_orca>"
  (cl:format cl:nil "std_msgs/Header header~%~%## 基本状态~%uint8 mission_state~%## 基本状态~%uint8 uav_id                ## 无人机编号~%## 是否到达目标点~%bool arrived_goal~%## 是否到达目标点~%bool arrived_all_goal~%## 目标位置~%float32[2] goal                 ## [m]~%## ORCA期望速度~%float32[2] vel_orca                 ## [m/s]~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rmtt_orca)))
  "Returns full string definition for message of type 'rmtt_orca"
  (cl:format cl:nil "std_msgs/Header header~%~%## 基本状态~%uint8 mission_state~%## 基本状态~%uint8 uav_id                ## 无人机编号~%## 是否到达目标点~%bool arrived_goal~%## 是否到达目标点~%bool arrived_all_goal~%## 目标位置~%float32[2] goal                 ## [m]~%## ORCA期望速度~%float32[2] vel_orca                 ## [m/s]~%~%~%~%~%~%    ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rmtt_orca>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vel_orca) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rmtt_orca>))
  "Converts a ROS message object to a list"
  (cl:list 'rmtt_orca
    (cl:cons ':header (header msg))
    (cl:cons ':mission_state (mission_state msg))
    (cl:cons ':uav_id (uav_id msg))
    (cl:cons ':arrived_goal (arrived_goal msg))
    (cl:cons ':arrived_all_goal (arrived_all_goal msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':vel_orca (vel_orca msg))
))
