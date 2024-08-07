;; Auto-generated. Do not edit!


(when (boundp 'sunray_msgs::rmtt_orca)
  (if (not (find-package "SUNRAY_MSGS"))
    (make-package "SUNRAY_MSGS"))
  (shadow 'rmtt_orca (find-package "SUNRAY_MSGS")))
(unless (find-package "SUNRAY_MSGS::RMTT_ORCA")
  (make-package "SUNRAY_MSGS::RMTT_ORCA"))

(in-package "ROS")
;;//! \htmlinclude rmtt_orca.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sunray_msgs::rmtt_orca
  :super ros::object
  :slots (_header _mission_state _uav_id _arrived_goal _arrived_all_goal _goal _vel_orca ))

(defmethod sunray_msgs::rmtt_orca
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mission_state __mission_state) 0)
    ((:uav_id __uav_id) 0)
    ((:arrived_goal __arrived_goal) nil)
    ((:arrived_all_goal __arrived_all_goal) nil)
    ((:goal __goal) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:vel_orca __vel_orca) (make-array 2 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _mission_state (round __mission_state))
   (setq _uav_id (round __uav_id))
   (setq _arrived_goal __arrived_goal)
   (setq _arrived_all_goal __arrived_all_goal)
   (setq _goal __goal)
   (setq _vel_orca __vel_orca)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mission_state
   (&optional __mission_state)
   (if __mission_state (setq _mission_state __mission_state)) _mission_state)
  (:uav_id
   (&optional __uav_id)
   (if __uav_id (setq _uav_id __uav_id)) _uav_id)
  (:arrived_goal
   (&optional (__arrived_goal :null))
   (if (not (eq __arrived_goal :null)) (setq _arrived_goal __arrived_goal)) _arrived_goal)
  (:arrived_all_goal
   (&optional (__arrived_all_goal :null))
   (if (not (eq __arrived_all_goal :null)) (setq _arrived_all_goal __arrived_all_goal)) _arrived_all_goal)
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:vel_orca
   (&optional __vel_orca)
   (if __vel_orca (setq _vel_orca __vel_orca)) _vel_orca)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _mission_state
    1
    ;; uint8 _uav_id
    1
    ;; bool _arrived_goal
    1
    ;; bool _arrived_all_goal
    1
    ;; float32[2] _goal
    (* 4    2)
    ;; float32[2] _vel_orca
    (* 4    2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _mission_state
       (write-byte _mission_state s)
     ;; uint8 _uav_id
       (write-byte _uav_id s)
     ;; bool _arrived_goal
       (if _arrived_goal (write-byte -1 s) (write-byte 0 s))
     ;; bool _arrived_all_goal
       (if _arrived_all_goal (write-byte -1 s) (write-byte 0 s))
     ;; float32[2] _goal
     (dotimes (i 2)
       (sys::poke (elt _goal i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[2] _vel_orca
     (dotimes (i 2)
       (sys::poke (elt _vel_orca i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _mission_state
     (setq _mission_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _uav_id
     (setq _uav_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _arrived_goal
     (setq _arrived_goal (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _arrived_all_goal
     (setq _arrived_all_goal (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[2] _goal
   (dotimes (i (length _goal))
     (setf (elt _goal i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[2] _vel_orca
   (dotimes (i (length _vel_orca))
     (setf (elt _vel_orca i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get sunray_msgs::rmtt_orca :md5sum-) "94cd23020ece232971ee6a1b721a7e0d")
(setf (get sunray_msgs::rmtt_orca :datatype-) "sunray_msgs/rmtt_orca")
(setf (get sunray_msgs::rmtt_orca :definition-)
      "std_msgs/Header header

## 基本状态
uint8 mission_state
## 基本状态
uint8 uav_id                ## 无人机编号
## 是否到达目标点
bool arrived_goal
## 是否到达目标点
bool arrived_all_goal
## 目标位置
float32[2] goal                 ## [m]
## ORCA期望速度
float32[2] vel_orca                 ## [m/s]





    

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

")



(provide :sunray_msgs/rmtt_orca "94cd23020ece232971ee6a1b721a7e0d")


