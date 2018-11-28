// Auto-generated. Do not edit!

// (in-package iv_slam_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class SubmapIndex {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory_id = null;
      this.submap_index = null;
      this.finishflag = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory_id')) {
        this.trajectory_id = initObj.trajectory_id
      }
      else {
        this.trajectory_id = 0;
      }
      if (initObj.hasOwnProperty('submap_index')) {
        this.submap_index = initObj.submap_index
      }
      else {
        this.submap_index = 0;
      }
      if (initObj.hasOwnProperty('finishflag')) {
        this.finishflag = initObj.finishflag
      }
      else {
        this.finishflag = false;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SubmapIndex
    // Serialize message field [trajectory_id]
    bufferOffset = _serializer.int32(obj.trajectory_id, buffer, bufferOffset);
    // Serialize message field [submap_index]
    bufferOffset = _serializer.int32(obj.submap_index, buffer, bufferOffset);
    // Serialize message field [finishflag]
    bufferOffset = _serializer.bool(obj.finishflag, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SubmapIndex
    let len;
    let data = new SubmapIndex(null);
    // Deserialize message field [trajectory_id]
    data.trajectory_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [submap_index]
    data.submap_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [finishflag]
    data.finishflag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 65;
  }

  static datatype() {
    // Returns string type for a message object
    return 'iv_slam_ros_msgs/SubmapIndex';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '85548cba5025a815b152574d9b381c6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 trajectory_id
    int32 submap_index
    bool finishflag
    geometry_msgs/Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
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
    const resolved = new SubmapIndex(null);
    if (msg.trajectory_id !== undefined) {
      resolved.trajectory_id = msg.trajectory_id;
    }
    else {
      resolved.trajectory_id = 0
    }

    if (msg.submap_index !== undefined) {
      resolved.submap_index = msg.submap_index;
    }
    else {
      resolved.submap_index = 0
    }

    if (msg.finishflag !== undefined) {
      resolved.finishflag = msg.finishflag;
    }
    else {
      resolved.finishflag = false
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = SubmapIndex;