## rmtt_base

Robomaster TT 驱动包


## Specification
|  Items  | Details|
|    :---:    |     :---:     |
| Model  | TLW004 |
| Weight  | 87 g |
| Max Speed  | 28.8 kph |
| Max Flight Time  | 13 min |
| Camera  | 2592×1936 |
| Battery  | LiPo |
| Capacity  | 1100 mAh |
| Voltage  | 3.8 V |


## RMTT Driver Topics
Published topics:
  - ~imu_data [sensor_msgs/Imu] 
  imu data. 

  - ~pose [geometry_msgs/PoseStamped] 
  Drone pose on the mission pad.

  - ~mission_pad_id [std_msgs/UInt8] 
  Mission pad id. 1 - 12.

  - ~image_raw/compressed [sensor_msgs/CompressedImage] 
  Compressed image topic.

  - ~image_raw [sensor_msgs/Image] 
  Image topic.

  - ~camera_info [sensor_msgs/CameraInfo] 
  RMTT camera info.

  - ~tof_btm [sensor_msgs/Range] 
  Bottom tof. Distance from drone to the ground.

  - ~tof_ext [sensor_msgs/Range] 
  External tof. Distance to the obstacle in front of the drone.

  - ~altitude [std_msgs/Float32] 
  Barometer readings.

  - ~battery [std_msgs/Float32] 
  Battery percentage.

Subscribed topics:
  - ~cmd_vel [geometry_msgs/Twist] 
  Command velocity. Only be effected after taking off.

  - ~takeoff [std_msgs/Empty] 
  Takeoff.

  - ~land [std_msgs/Empty]
  Land.

  - ~flip [std_msgs/Empty] 
  Flip.

  - ~led [std_msgs/ColorRGBA] 
  Led on top of the external module.

  - ~mled [std_msgs/String] 
  Dot matrix display. Showing charactor or string.

Services:
  - ~set_downvision [std_srvs/SetBool]
  Change the front cam to downward cam

  - ~set_hdmap
  Change the positioning to HD map mode. Can be used with the mission pad or HD map.
