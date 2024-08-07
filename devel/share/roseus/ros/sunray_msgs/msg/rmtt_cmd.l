;; Auto-generated. Do not edit!


(when (boundp 'sunray_msgs::rmtt_cmd)
  (if (not (find-package "SUNRAY_MSGS"))
    (make-package "SUNRAY_MSGS"))
  (shadow 'rmtt_cmd (find-package "SUNRAY_MSGS")))
(unless (find-package "SUNRAY_MSGS::RMTT_CMD")
  (make-package "SUNRAY_MSGS::RMTT_CMD"))

(in-package "ROS")
;;//! \htmlinclude rmtt_cmd.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sunray_msgs::rmtt_cmd
  :super ros::object
  :slots (_header _agent_id _control_state _desired_pos _desired_yaw _desired_vel ))

(defmethod sunray_msgs::rmtt_cmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:agent_id __agent_id) 0)
    ((:control_state __control_state) 0)
    ((:desired_pos __desired_pos) (instance geometry_msgs::Point :init))
    ((:desired_yaw __desired_yaw) 0.0)
    ((:desired_vel __desired_vel) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _agent_id (round __agent_id))
   (setq _control_state (round __control_state))
   (setq _desired_pos __desired_pos)
   (setq _desired_yaw (float __desired_yaw))
   (setq _desired_vel __desired_vel)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:agent_id
   (&optional __agent_id)
   (if __agent_id (setq _agent_id __agent_id)) _agent_id)
  (:control_state
   (&optional __control_state)
   (if __control_state (setq _control_state __control_state)) _control_state)
  (:desired_pos
   (&rest __desired_pos)
   (if (keywordp (car __desired_pos))
       (send* _desired_pos __desired_pos)
     (progn
       (if __desired_pos (setq _desired_pos (car __desired_pos)))
       _desired_pos)))
  (:desired_yaw
   (&optional __desired_yaw)
   (if __desired_yaw (setq _desired_yaw __desired_yaw)) _desired_yaw)
  (:desired_vel
   (&rest __desired_vel)
   (if (keywordp (car __desired_vel))
       (send* _desired_vel __desired_vel)
     (progn
       (if __desired_vel (setq _desired_vel (car __desired_vel)))
       _desired_vel)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _agent_id
    1
    ;; uint8 _control_state
    1
    ;; geometry_msgs/Point _desired_pos
    (send _desired_pos :serialization-length)
    ;; float32 _desired_yaw
    4
    ;; geometry_msgs/Twist _desired_vel
    (send _desired_vel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _agent_id
       (write-byte _agent_id s)
     ;; uint8 _control_state
       (write-byte _control_state s)
     ;; geometry_msgs/Point _desired_pos
       (send _desired_pos :serialize s)
     ;; float32 _desired_yaw
       (sys::poke _desired_yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Twist _desired_vel
       (send _desired_vel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _agent_id
     (setq _agent_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _control_state
     (setq _control_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point _desired_pos
     (send _desired_pos :deserialize buf ptr-) (incf ptr- (send _desired_pos :serialization-length))
   ;; float32 _desired_yaw
     (setq _desired_yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Twist _desired_vel
     (send _desired_vel :deserialize buf ptr-) (incf ptr- (send _desired_vel :serialization-length))
   ;;
   self)
  )

(setf (get sunray_msgs::rmtt_cmd :md5sum-) "a1941fe70865cfa39c036a2c1ad0339a")
(setf (get sunray_msgs::rmtt_cmd :datatype-) "sunray_msgs/rmtt_cmd")
(setf (get sunray_msgs::rmtt_cmd :definition-)
      "std_msgs/Header header

## 当指定ID的时候，只有指定的ID响应
uint8 agent_id  

## 状态机指令
uint8 control_state   

## 期望位置、偏航角 -> 对应POS_CONTROL模式
geometry_msgs/Point desired_pos    ## [m]
float32 desired_yaw                 ## [rad]

## 期望速度 -> 对应VEL_CONTROL模式
geometry_msgs/Twist desired_vel






    

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :sunray_msgs/rmtt_cmd "a1941fe70865cfa39c036a2c1ad0339a")


