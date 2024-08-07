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

class agent_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.agent_type = null;
      this.agent_id = null;
      this.agent_ip = null;
      this.connected = null;
      this.odom_valid = null;
      this.pos = null;
      this.vel = null;
      this.att = null;
      this.attitude_q = null;
      this.battery = null;
      this.control_state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('agent_type')) {
        this.agent_type = initObj.agent_type
      }
      else {
        this.agent_type = 0;
      }
      if (initObj.hasOwnProperty('agent_id')) {
        this.agent_id = initObj.agent_id
      }
      else {
        this.agent_id = 0;
      }
      if (initObj.hasOwnProperty('agent_ip')) {
        this.agent_ip = initObj.agent_ip
      }
      else {
        this.agent_ip = '';
      }
      if (initObj.hasOwnProperty('connected')) {
        this.connected = initObj.connected
      }
      else {
        this.connected = false;
      }
      if (initObj.hasOwnProperty('odom_valid')) {
        this.odom_valid = initObj.odom_valid
      }
      else {
        this.odom_valid = false;
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('att')) {
        this.att = initObj.att
      }
      else {
        this.att = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('attitude_q')) {
        this.attitude_q = initObj.attitude_q
      }
      else {
        this.attitude_q = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('battery')) {
        this.battery = initObj.battery
      }
      else {
        this.battery = 0.0;
      }
      if (initObj.hasOwnProperty('control_state')) {
        this.control_state = initObj.control_state
      }
      else {
        this.control_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type agent_state
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [agent_type]
    bufferOffset = _serializer.uint8(obj.agent_type, buffer, bufferOffset);
    // Serialize message field [agent_id]
    bufferOffset = _serializer.uint8(obj.agent_id, buffer, bufferOffset);
    // Serialize message field [agent_ip]
    bufferOffset = _serializer.string(obj.agent_ip, buffer, bufferOffset);
    // Serialize message field [connected]
    bufferOffset = _serializer.bool(obj.connected, buffer, bufferOffset);
    // Serialize message field [odom_valid]
    bufferOffset = _serializer.bool(obj.odom_valid, buffer, bufferOffset);
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 3) {
      throw new Error('Unable to serialize array field pos - length must be 3')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float32(obj.pos, buffer, bufferOffset, 3);
    // Check that the constant length array field [vel] has the right length
    if (obj.vel.length !== 3) {
      throw new Error('Unable to serialize array field vel - length must be 3')
    }
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float32(obj.vel, buffer, bufferOffset, 3);
    // Check that the constant length array field [att] has the right length
    if (obj.att.length !== 3) {
      throw new Error('Unable to serialize array field att - length must be 3')
    }
    // Serialize message field [att]
    bufferOffset = _arraySerializer.float32(obj.att, buffer, bufferOffset, 3);
    // Serialize message field [attitude_q]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.attitude_q, buffer, bufferOffset);
    // Serialize message field [battery]
    bufferOffset = _serializer.float32(obj.battery, buffer, bufferOffset);
    // Serialize message field [control_state]
    bufferOffset = _serializer.uint8(obj.control_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type agent_state
    let len;
    let data = new agent_state(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [agent_type]
    data.agent_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [agent_id]
    data.agent_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [agent_ip]
    data.agent_ip = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [connected]
    data.connected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [odom_valid]
    data.odom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [att]
    data.att = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [attitude_q]
    data.attitude_q = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [battery]
    data.battery = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [control_state]
    data.control_state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.agent_ip);
    return length + 81;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sunray_msgs/agent_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'adb46b835ede27744c5ee8dbd1eaf160';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new agent_state(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.agent_type !== undefined) {
      resolved.agent_type = msg.agent_type;
    }
    else {
      resolved.agent_type = 0
    }

    if (msg.agent_id !== undefined) {
      resolved.agent_id = msg.agent_id;
    }
    else {
      resolved.agent_id = 0
    }

    if (msg.agent_ip !== undefined) {
      resolved.agent_ip = msg.agent_ip;
    }
    else {
      resolved.agent_ip = ''
    }

    if (msg.connected !== undefined) {
      resolved.connected = msg.connected;
    }
    else {
      resolved.connected = false
    }

    if (msg.odom_valid !== undefined) {
      resolved.odom_valid = msg.odom_valid;
    }
    else {
      resolved.odom_valid = false
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(3).fill(0)
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = new Array(3).fill(0)
    }

    if (msg.att !== undefined) {
      resolved.att = msg.att;
    }
    else {
      resolved.att = new Array(3).fill(0)
    }

    if (msg.attitude_q !== undefined) {
      resolved.attitude_q = geometry_msgs.msg.Quaternion.Resolve(msg.attitude_q)
    }
    else {
      resolved.attitude_q = new geometry_msgs.msg.Quaternion()
    }

    if (msg.battery !== undefined) {
      resolved.battery = msg.battery;
    }
    else {
      resolved.battery = 0.0
    }

    if (msg.control_state !== undefined) {
      resolved.control_state = msg.control_state;
    }
    else {
      resolved.control_state = 0
    }

    return resolved;
    }
};

// Constants for message
agent_state.Constants = {
  RMTT: 0,
  TIANBOT: 1,
  WHEELTEC: 2,
  SIKONG: 3,
}

module.exports = agent_state;
