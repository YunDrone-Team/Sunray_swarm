;; Auto-generated. Do not edit!


(when (boundp 'sunray_msgs::agent_state)
  (if (not (find-package "SUNRAY_MSGS"))
    (make-package "SUNRAY_MSGS"))
  (shadow 'agent_state (find-package "SUNRAY_MSGS")))
(unless (find-package "SUNRAY_MSGS::AGENT_STATE")
  (make-package "SUNRAY_MSGS::AGENT_STATE"))

(in-package "ROS")
;;//! \htmlinclude agent_state.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*RMTT*" (find-package "SUNRAY_MSGS::AGENT_STATE"))
(shadow '*RMTT* (find-package "SUNRAY_MSGS::AGENT_STATE"))
(defconstant sunray_msgs::agent_state::*RMTT* 0)
(intern "*TIANBOT*" (find-package "SUNRAY_MSGS::AGENT_STATE"))
(shadow '*TIANBOT* (find-package "SUNRAY_MSGS::AGENT_STATE"))
(defconstant sunray_msgs::agent_state::*TIANBOT* 1)
(intern "*WHEELTEC*" (find-package "SUNRAY_MSGS::AGENT_STATE"))
(shadow '*WHEELTEC* (find-package "SUNRAY_MSGS::AGENT_STATE"))
(defconstant sunray_msgs::agent_state::*WHEELTEC* 2)
(intern "*SIKONG*" (find-package "SUNRAY_MSGS::AGENT_STATE"))
(shadow '*SIKONG* (find-package "SUNRAY_MSGS::AGENT_STATE"))
(defconstant sunray_msgs::agent_state::*SIKONG* 3)

(defun sunray_msgs::agent_state-to-symbol (const)
  (cond
        ((= const 0) 'sunray_msgs::agent_state::*RMTT*)
        ((= const 1) 'sunray_msgs::agent_state::*TIANBOT*)
        ((= const 2) 'sunray_msgs::agent_state::*WHEELTEC*)
        ((= const 3) 'sunray_msgs::agent_state::*SIKONG*)
        (t nil)))

(defclass sunray_msgs::agent_state
  :super ros::object
  :slots (_header _agent_type _agent_id _agent_ip _connected _odom_valid _pos _vel _att _attitude_q _battery _control_state ))

(defmethod sunray_msgs::agent_state
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:agent_type __agent_type) 0)
    ((:agent_id __agent_id) 0)
    ((:agent_ip __agent_ip) "")
    ((:connected __connected) nil)
    ((:odom_valid __odom_valid) nil)
    ((:pos __pos) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:att __att) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:attitude_q __attitude_q) (instance geometry_msgs::Quaternion :init))
    ((:battery __battery) 0.0)
    ((:control_state __control_state) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _agent_type (round __agent_type))
   (setq _agent_id (round __agent_id))
   (setq _agent_ip (string __agent_ip))
   (setq _connected __connected)
   (setq _odom_valid __odom_valid)
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _att __att)
   (setq _attitude_q __attitude_q)
   (setq _battery (float __battery))
   (setq _control_state (round __control_state))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:agent_type
   (&optional __agent_type)
   (if __agent_type (setq _agent_type __agent_type)) _agent_type)
  (:agent_id
   (&optional __agent_id)
   (if __agent_id (setq _agent_id __agent_id)) _agent_id)
  (:agent_ip
   (&optional __agent_ip)
   (if __agent_ip (setq _agent_ip __agent_ip)) _agent_ip)
  (:connected
   (&optional (__connected :null))
   (if (not (eq __connected :null)) (setq _connected __connected)) _connected)
  (:odom_valid
   (&optional (__odom_valid :null))
   (if (not (eq __odom_valid :null)) (setq _odom_valid __odom_valid)) _odom_valid)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:att
   (&optional __att)
   (if __att (setq _att __att)) _att)
  (:attitude_q
   (&rest __attitude_q)
   (if (keywordp (car __attitude_q))
       (send* _attitude_q __attitude_q)
     (progn
       (if __attitude_q (setq _attitude_q (car __attitude_q)))
       _attitude_q)))
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:control_state
   (&optional __control_state)
   (if __control_state (setq _control_state __control_state)) _control_state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _agent_type
    1
    ;; uint8 _agent_id
    1
    ;; string _agent_ip
    4 (length _agent_ip)
    ;; bool _connected
    1
    ;; bool _odom_valid
    1
    ;; float32[3] _pos
    (* 4    3)
    ;; float32[3] _vel
    (* 4    3)
    ;; float32[3] _att
    (* 4    3)
    ;; geometry_msgs/Quaternion _attitude_q
    (send _attitude_q :serialization-length)
    ;; float32 _battery
    4
    ;; uint8 _control_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _agent_type
       (write-byte _agent_type s)
     ;; uint8 _agent_id
       (write-byte _agent_id s)
     ;; string _agent_ip
       (write-long (length _agent_ip) s) (princ _agent_ip s)
     ;; bool _connected
       (if _connected (write-byte -1 s) (write-byte 0 s))
     ;; bool _odom_valid
       (if _odom_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32[3] _pos
     (dotimes (i 3)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _vel
     (dotimes (i 3)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _att
     (dotimes (i 3)
       (sys::poke (elt _att i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; geometry_msgs/Quaternion _attitude_q
       (send _attitude_q :serialize s)
     ;; float32 _battery
       (sys::poke _battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _control_state
       (write-byte _control_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _agent_type
     (setq _agent_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _agent_id
     (setq _agent_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _agent_ip
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _agent_ip (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _connected
     (setq _connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _odom_valid
     (setq _odom_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[3] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _att
   (dotimes (i (length _att))
     (setf (elt _att i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; geometry_msgs/Quaternion _attitude_q
     (send _attitude_q :deserialize buf ptr-) (incf ptr- (send _attitude_q :serialization-length))
   ;; float32 _battery
     (setq _battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _control_state
     (setq _control_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get sunray_msgs::agent_state :md5sum-) "adb46b835ede27744c5ee8dbd1eaf160")
(setf (get sunray_msgs::agent_state :datatype-) "sunray_msgs/agent_state")
(setf (get sunray_msgs::agent_state :definition-)
      "std_msgs/Header header

## 基本状态
uint8 agent_type              ## 智能体类型
uint8 agent_id                ## 智能体编号
string agent_ip                 ## 智能体IP
bool connected                ## 是否连接上智能体驱动
bool odom_valid               ## 是否收到动捕数据

## 智能体位置、速度、姿态
float32[3] pos                 ## [m]
float32[3] vel                 ## [m/s]
float32[3] att                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数

## 智能体电池状态
float32 battery                ## [0-1]

## 智能体控制状态
uint8 control_state

## agent_type枚举
uint8 RMTT = 0
uint8 TIANBOT = 1
uint8 WHEELTEC = 2
uint8 SIKONG = 3
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
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :sunray_msgs/agent_state "adb46b835ede27744c5ee8dbd1eaf160")


