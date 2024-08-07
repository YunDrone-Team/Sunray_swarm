;; Auto-generated. Do not edit!


(when (boundp 'sunray_msgs::station_cmd)
  (if (not (find-package "SUNRAY_MSGS"))
    (make-package "SUNRAY_MSGS"))
  (shadow 'station_cmd (find-package "SUNRAY_MSGS")))
(unless (find-package "SUNRAY_MSGS::STATION_CMD")
  (make-package "SUNRAY_MSGS::STATION_CMD"))

(in-package "ROS")
;;//! \htmlinclude station_cmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sunray_msgs::station_cmd
  :super ros::object
  :slots (_header _uav_id _mission_state _scenario_id ))

(defmethod sunray_msgs::station_cmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:uav_id __uav_id) 0)
    ((:mission_state __mission_state) 0)
    ((:scenario_id __scenario_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _uav_id (round __uav_id))
   (setq _mission_state (round __mission_state))
   (setq _scenario_id (round __scenario_id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:uav_id
   (&optional __uav_id)
   (if __uav_id (setq _uav_id __uav_id)) _uav_id)
  (:mission_state
   (&optional __mission_state)
   (if __mission_state (setq _mission_state __mission_state)) _mission_state)
  (:scenario_id
   (&optional __scenario_id)
   (if __scenario_id (setq _scenario_id __scenario_id)) _scenario_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _uav_id
    1
    ;; uint8 _mission_state
    1
    ;; uint8 _scenario_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _uav_id
       (write-byte _uav_id s)
     ;; uint8 _mission_state
       (write-byte _mission_state s)
     ;; uint8 _scenario_id
       (write-byte _scenario_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _uav_id
     (setq _uav_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _mission_state
     (setq _mission_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _scenario_id
     (setq _scenario_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get sunray_msgs::station_cmd :md5sum-) "f9fbf99fef9ff42ede4e5c27986fb1df")
(setf (get sunray_msgs::station_cmd :datatype-) "sunray_msgs/station_cmd")
(setf (get sunray_msgs::station_cmd :definition-)
      "std_msgs/Header header

## 对于起飞、降落、悬停、ORCA_RUN指令：当指定ID的时候，只有指定的ID响应
## 对于其他指令，uav_id设置为99，所有飞机响应该状态  
uint8 uav_id  

## 状态机指令
uint8 mission_state   
uint8 scenario_id           
 





    

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



(provide :sunray_msgs/station_cmd "f9fbf99fef9ff42ede4e5c27986fb1df")


