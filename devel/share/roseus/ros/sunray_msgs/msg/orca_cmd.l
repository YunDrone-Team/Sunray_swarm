;; Auto-generated. Do not edit!


(when (boundp 'sunray_msgs::orca_cmd)
  (if (not (find-package "SUNRAY_MSGS"))
    (make-package "SUNRAY_MSGS"))
  (shadow 'orca_cmd (find-package "SUNRAY_MSGS")))
(unless (find-package "SUNRAY_MSGS::ORCA_CMD")
  (make-package "SUNRAY_MSGS::ORCA_CMD"))

(in-package "ROS")
;;//! \htmlinclude orca_cmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sunray_msgs::orca_cmd
  :super ros::object
  :slots (_header _orca_cmd _scenario_id ))

(defmethod sunray_msgs::orca_cmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:orca_cmd __orca_cmd) 0)
    ((:scenario_id __scenario_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _orca_cmd (round __orca_cmd))
   (setq _scenario_id (round __scenario_id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:orca_cmd
   (&optional __orca_cmd)
   (if __orca_cmd (setq _orca_cmd __orca_cmd)) _orca_cmd)
  (:scenario_id
   (&optional __scenario_id)
   (if __scenario_id (setq _scenario_id __scenario_id)) _scenario_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _orca_cmd
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
     ;; uint8 _orca_cmd
       (write-byte _orca_cmd s)
     ;; uint8 _scenario_id
       (write-byte _scenario_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _orca_cmd
     (setq _orca_cmd (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _scenario_id
     (setq _scenario_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get sunray_msgs::orca_cmd :md5sum-) "c1d519fdc68c652710d2bdf69a489838")
(setf (get sunray_msgs::orca_cmd :datatype-) "sunray_msgs/orca_cmd")
(setf (get sunray_msgs::orca_cmd :definition-)
      "std_msgs/Header header

## 状态机指令
uint8 orca_cmd   
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



(provide :sunray_msgs/orca_cmd "c1d519fdc68c652710d2bdf69a489838")


