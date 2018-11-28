// Auto-generated. Do not edit!

// (in-package control_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ModeReport = require('./ModeReport.js');
let SteerReport = require('./SteerReport.js');
let GearReport = require('./GearReport.js');
let BrakeReport = require('./BrakeReport.js');
let ThrottleReport = require('./ThrottleReport.js');
let SpeedReport = require('./SpeedReport.js');
let HMIReport = require('./HMIReport.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GetECUReport {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.mode = null;
      this.steer_cur = null;
      this.shift_cur = null;
      this.shift1_cur = null;
      this.brake_cur = null;
      this.throttle_cur = null;
      this.speed = null;
      this.manual = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = new ModeReport();
      }
      if (initObj.hasOwnProperty('steer_cur')) {
        this.steer_cur = initObj.steer_cur
      }
      else {
        this.steer_cur = new SteerReport();
      }
      if (initObj.hasOwnProperty('shift_cur')) {
        this.shift_cur = initObj.shift_cur
      }
      else {
        this.shift_cur = new GearReport();
      }
      if (initObj.hasOwnProperty('shift1_cur')) {
        this.shift1_cur = initObj.shift1_cur
      }
      else {
        this.shift1_cur = new GearReport();
      }
      if (initObj.hasOwnProperty('brake_cur')) {
        this.brake_cur = initObj.brake_cur
      }
      else {
        this.brake_cur = new BrakeReport();
      }
      if (initObj.hasOwnProperty('throttle_cur')) {
        this.throttle_cur = initObj.throttle_cur
      }
      else {
        this.throttle_cur = new ThrottleReport();
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = new SpeedReport();
      }
      if (initObj.hasOwnProperty('manual')) {
        this.manual = initObj.manual
      }
      else {
        this.manual = new HMIReport();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetECUReport
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = ModeReport.serialize(obj.mode, buffer, bufferOffset);
    // Serialize message field [steer_cur]
    bufferOffset = SteerReport.serialize(obj.steer_cur, buffer, bufferOffset);
    // Serialize message field [shift_cur]
    bufferOffset = GearReport.serialize(obj.shift_cur, buffer, bufferOffset);
    // Serialize message field [shift1_cur]
    bufferOffset = GearReport.serialize(obj.shift1_cur, buffer, bufferOffset);
    // Serialize message field [brake_cur]
    bufferOffset = BrakeReport.serialize(obj.brake_cur, buffer, bufferOffset);
    // Serialize message field [throttle_cur]
    bufferOffset = ThrottleReport.serialize(obj.throttle_cur, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = SpeedReport.serialize(obj.speed, buffer, bufferOffset);
    // Serialize message field [manual]
    bufferOffset = HMIReport.serialize(obj.manual, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetECUReport
    let len;
    let data = new GetECUReport(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = ModeReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [steer_cur]
    data.steer_cur = SteerReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [shift_cur]
    data.shift_cur = GearReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [shift1_cur]
    data.shift1_cur = GearReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [brake_cur]
    data.brake_cur = BrakeReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [throttle_cur]
    data.throttle_cur = ThrottleReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = SpeedReport.deserialize(buffer, bufferOffset);
    // Deserialize message field [manual]
    data.manual = HMIReport.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += ModeReport.getMessageSize(object.mode);
    length += SteerReport.getMessageSize(object.steer_cur);
    length += GearReport.getMessageSize(object.shift_cur);
    length += GearReport.getMessageSize(object.shift1_cur);
    length += BrakeReport.getMessageSize(object.brake_cur);
    length += ThrottleReport.getMessageSize(object.throttle_cur);
    length += SpeedReport.getMessageSize(object.speed);
    length += HMIReport.getMessageSize(object.manual);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/GetECUReport';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83102858bdab6b0e68d7ca00281fe1a4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    control_msgs/ModeReport mode
    
    control_msgs/SteerReport steer_cur
    
    control_msgs/GearReport shift_cur
    control_msgs/GearReport shift1_cur
    
    control_msgs/BrakeReport brake_cur
    control_msgs/ThrottleReport throttle_cur # include engine_status
    
    control_msgs/SpeedReport speed
    
    control_msgs/HMIReport manual
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: control_msgs/ModeReport
    Header header
    
    int32 auto_mode
    
    int32 speed_mode
    bool throttle_enable
    bool brake_enable
    
    int32 steer_mode
    int32 gear_mode
    
    
    ================================================================================
    MSG: control_msgs/SteerReport
    Header header
    float32 steer
    
    ================================================================================
    MSG: control_msgs/GearReport
    Header header
    uint8 gear
    
    ================================================================================
    MSG: control_msgs/BrakeReport
    Header header
    
    float32 brake_ESC_left
    float32 brake_ESC_right
    
    control_msgs/WheelStateReport brake_wheel
    
    float32 brake_pedal
    
    ================================================================================
    MSG: control_msgs/WheelStateReport
    Header header
    
    float32 front_left
    float32 front_right
    float32 rear_left
    float32 rear_right
    
    ================================================================================
    MSG: control_msgs/ThrottleReport
    Header header
    float32 throttle
    control_msgs/EngineReport engine_status
    float32 throttle_pedal
    
    ================================================================================
    MSG: control_msgs/EngineReport
    Header header
    float32 engine_rpm
    float32 engine_load
    
    ================================================================================
    MSG: control_msgs/SpeedReport
    Header header
    
    control_msgs/WheelStateReport speed_wheel
    geometry_msgs/Twist velocity
    
    
    
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
    ================================================================================
    MSG: control_msgs/HMIReport
    Header header
    
    bool manual_estop
    bool is_human_brake 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetECUReport(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.mode !== undefined) {
      resolved.mode = ModeReport.Resolve(msg.mode)
    }
    else {
      resolved.mode = new ModeReport()
    }

    if (msg.steer_cur !== undefined) {
      resolved.steer_cur = SteerReport.Resolve(msg.steer_cur)
    }
    else {
      resolved.steer_cur = new SteerReport()
    }

    if (msg.shift_cur !== undefined) {
      resolved.shift_cur = GearReport.Resolve(msg.shift_cur)
    }
    else {
      resolved.shift_cur = new GearReport()
    }

    if (msg.shift1_cur !== undefined) {
      resolved.shift1_cur = GearReport.Resolve(msg.shift1_cur)
    }
    else {
      resolved.shift1_cur = new GearReport()
    }

    if (msg.brake_cur !== undefined) {
      resolved.brake_cur = BrakeReport.Resolve(msg.brake_cur)
    }
    else {
      resolved.brake_cur = new BrakeReport()
    }

    if (msg.throttle_cur !== undefined) {
      resolved.throttle_cur = ThrottleReport.Resolve(msg.throttle_cur)
    }
    else {
      resolved.throttle_cur = new ThrottleReport()
    }

    if (msg.speed !== undefined) {
      resolved.speed = SpeedReport.Resolve(msg.speed)
    }
    else {
      resolved.speed = new SpeedReport()
    }

    if (msg.manual !== undefined) {
      resolved.manual = HMIReport.Resolve(msg.manual)
    }
    else {
      resolved.manual = new HMIReport()
    }

    return resolved;
    }
};

module.exports = GetECUReport;
