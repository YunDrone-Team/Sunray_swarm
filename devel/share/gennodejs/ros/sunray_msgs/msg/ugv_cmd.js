// Auto-generated. Do not edit!

// (in-package sunray_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ugv_cmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.agent_id = null;
      this.control_state = null;
      this.desired_pos = null;
      this.desired_yaw = null;
      this.desired_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('agent_id')) {
        this.agent_id = initObj.agent_id
      }
      else {
        this.agent_id = 0;
      }
      if (initObj.hasOwnProperty('control_state')) {
        this.control_state = initObj.control_state
      }
      else {
        this.control_state = 0;
      }
      if (initObj.hasOwnProperty('desired_pos')) {
        this.desired_pos = initObj.desired_pos
      }
      else {
        this.desired_pos = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('desired_yaw')) {
        this.desired_yaw = initObj.desired_yaw
      }
      else {
        this.desired_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('desired_vel')) {
        this.desired_vel = initObj.desired_vel
      }
      else {
        this.desired_vel = new geometry_msgs.msg.Twist();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ugv_cmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [agent_id]
    bufferOffset = _serializer.uint8(obj.agent_id, buffer, bufferOffset);
    // Serialize message field [control_state]
    bufferOffset = _serializer.uint8(obj.control_state, buffer, bufferOffset);
    // Serialize message field [desired_pos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.desired_pos, buffer, bufferOffset);
    // Serialize message field [desired_yaw]
    bufferOffset = _serializer.float32(obj.desired_yaw, buffer, bufferOffset);
    // Serialize message field [desired_vel]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.desired_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ugv_cmd
    let len;
    let data = new ugv_cmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [agent_id]
    data.agent_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [control_state]
    data.control_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [desired_pos]
    data.desired_pos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [desired_yaw]
    data.desired_yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desired_vel]
    data.desired_vel = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 78;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sunray_msgs/ugv_cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a1941fe70865cfa39c036a2c1ad0339a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ugv_cmd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.agent_id !== undefined) {
      resolved.agent_id = msg.agent_id;
    }
    else {
      resolved.agent_id = 0
    }

    if (msg.control_state !== undefined) {
      resolved.control_state = msg.control_state;
    }
    else {
      resolved.control_state = 0
    }

    if (msg.desired_pos !== undefined) {
      resolved.desired_pos = geometry_msgs.msg.Point.Resolve(msg.desired_pos)
    }
    else {
      resolved.desired_pos = new geometry_msgs.msg.Point()
    }

    if (msg.desired_yaw !== undefined) {
      resolved.desired_yaw = msg.desired_yaw;
    }
    else {
      resolved.desired_yaw = 0.0
    }

    if (msg.desired_vel !== undefined) {
      resolved.desired_vel = geometry_msgs.msg.Twist.Resolve(msg.desired_vel)
    }
    else {
      resolved.desired_vel = new geometry_msgs.msg.Twist()
    }

    return resolved;
    }
};

module.exports = ugv_cmd;
