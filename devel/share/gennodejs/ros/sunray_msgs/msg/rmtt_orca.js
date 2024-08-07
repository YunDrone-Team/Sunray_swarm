// Auto-generated. Do not edit!

// (in-package sunray_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class rmtt_orca {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.mission_state = null;
      this.uav_id = null;
      this.arrived_goal = null;
      this.arrived_all_goal = null;
      this.goal = null;
      this.vel_orca = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('mission_state')) {
        this.mission_state = initObj.mission_state
      }
      else {
        this.mission_state = 0;
      }
      if (initObj.hasOwnProperty('uav_id')) {
        this.uav_id = initObj.uav_id
      }
      else {
        this.uav_id = 0;
      }
      if (initObj.hasOwnProperty('arrived_goal')) {
        this.arrived_goal = initObj.arrived_goal
      }
      else {
        this.arrived_goal = false;
      }
      if (initObj.hasOwnProperty('arrived_all_goal')) {
        this.arrived_all_goal = initObj.arrived_all_goal
      }
      else {
        this.arrived_all_goal = false;
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('vel_orca')) {
        this.vel_orca = initObj.vel_orca
      }
      else {
        this.vel_orca = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rmtt_orca
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [mission_state]
    bufferOffset = _serializer.uint8(obj.mission_state, buffer, bufferOffset);
    // Serialize message field [uav_id]
    bufferOffset = _serializer.uint8(obj.uav_id, buffer, bufferOffset);
    // Serialize message field [arrived_goal]
    bufferOffset = _serializer.bool(obj.arrived_goal, buffer, bufferOffset);
    // Serialize message field [arrived_all_goal]
    bufferOffset = _serializer.bool(obj.arrived_all_goal, buffer, bufferOffset);
    // Check that the constant length array field [goal] has the right length
    if (obj.goal.length !== 2) {
      throw new Error('Unable to serialize array field goal - length must be 2')
    }
    // Serialize message field [goal]
    bufferOffset = _arraySerializer.float32(obj.goal, buffer, bufferOffset, 2);
    // Check that the constant length array field [vel_orca] has the right length
    if (obj.vel_orca.length !== 2) {
      throw new Error('Unable to serialize array field vel_orca - length must be 2')
    }
    // Serialize message field [vel_orca]
    bufferOffset = _arraySerializer.float32(obj.vel_orca, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rmtt_orca
    let len;
    let data = new rmtt_orca(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [mission_state]
    data.mission_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [uav_id]
    data.uav_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [arrived_goal]
    data.arrived_goal = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [arrived_all_goal]
    data.arrived_all_goal = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [vel_orca]
    data.vel_orca = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sunray_msgs/rmtt_orca';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '94cd23020ece232971ee6a1b721a7e0d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rmtt_orca(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.mission_state !== undefined) {
      resolved.mission_state = msg.mission_state;
    }
    else {
      resolved.mission_state = 0
    }

    if (msg.uav_id !== undefined) {
      resolved.uav_id = msg.uav_id;
    }
    else {
      resolved.uav_id = 0
    }

    if (msg.arrived_goal !== undefined) {
      resolved.arrived_goal = msg.arrived_goal;
    }
    else {
      resolved.arrived_goal = false
    }

    if (msg.arrived_all_goal !== undefined) {
      resolved.arrived_all_goal = msg.arrived_all_goal;
    }
    else {
      resolved.arrived_all_goal = false
    }

    if (msg.goal !== undefined) {
      resolved.goal = msg.goal;
    }
    else {
      resolved.goal = new Array(2).fill(0)
    }

    if (msg.vel_orca !== undefined) {
      resolved.vel_orca = msg.vel_orca;
    }
    else {
      resolved.vel_orca = new Array(2).fill(0)
    }

    return resolved;
    }
};

module.exports = rmtt_orca;
