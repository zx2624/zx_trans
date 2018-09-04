// Auto-generated. Do not edit!

// (in-package object_detector_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class laser_electronic_result {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.frameID = null;
      this.targetType = null;
      this.confidence = null;
      this.topleftX = null;
      this.topleftY = null;
      this.bottomrightX = null;
      this.bottomrightY = null;
      this.distance = null;
      this.image_data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('frameID')) {
        this.frameID = initObj.frameID
      }
      else {
        this.frameID = 0;
      }
      if (initObj.hasOwnProperty('targetType')) {
        this.targetType = initObj.targetType
      }
      else {
        this.targetType = 0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0;
      }
      if (initObj.hasOwnProperty('topleftX')) {
        this.topleftX = initObj.topleftX
      }
      else {
        this.topleftX = 0;
      }
      if (initObj.hasOwnProperty('topleftY')) {
        this.topleftY = initObj.topleftY
      }
      else {
        this.topleftY = 0;
      }
      if (initObj.hasOwnProperty('bottomrightX')) {
        this.bottomrightX = initObj.bottomrightX
      }
      else {
        this.bottomrightX = 0;
      }
      if (initObj.hasOwnProperty('bottomrightY')) {
        this.bottomrightY = initObj.bottomrightY
      }
      else {
        this.bottomrightY = 0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0;
      }
      if (initObj.hasOwnProperty('image_data')) {
        this.image_data = initObj.image_data
      }
      else {
        this.image_data = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type laser_electronic_result
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [frameID]
    bufferOffset = _serializer.uint32(obj.frameID, buffer, bufferOffset);
    // Serialize message field [targetType]
    bufferOffset = _serializer.uint8(obj.targetType, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.uint16(obj.confidence, buffer, bufferOffset);
    // Serialize message field [topleftX]
    bufferOffset = _serializer.uint16(obj.topleftX, buffer, bufferOffset);
    // Serialize message field [topleftY]
    bufferOffset = _serializer.uint16(obj.topleftY, buffer, bufferOffset);
    // Serialize message field [bottomrightX]
    bufferOffset = _serializer.uint16(obj.bottomrightX, buffer, bufferOffset);
    // Serialize message field [bottomrightY]
    bufferOffset = _serializer.uint16(obj.bottomrightY, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.uint16(obj.distance, buffer, bufferOffset);
    // Serialize message field [image_data]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.image_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type laser_electronic_result
    let len;
    let data = new laser_electronic_result(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [frameID]
    data.frameID = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [targetType]
    data.targetType = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [topleftX]
    data.topleftX = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [topleftY]
    data.topleftY = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [bottomrightX]
    data.bottomrightX = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [bottomrightY]
    data.bottomrightY = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [image_data]
    data.image_data = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.Image.getMessageSize(object.image_data);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_detector_msgs/laser_electronic_result';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28ce18d3994a9e82dc4a7dc982988a3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint32 frameID
    uint8  targetType
    uint16 confidence
    uint16 topleftX
    uint16 topleftY
    uint16 bottomrightX
    uint16 bottomrightY
    uint16 distance
    sensor_msgs/Image  image_data
    
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
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of cameara
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new laser_electronic_result(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.frameID !== undefined) {
      resolved.frameID = msg.frameID;
    }
    else {
      resolved.frameID = 0
    }

    if (msg.targetType !== undefined) {
      resolved.targetType = msg.targetType;
    }
    else {
      resolved.targetType = 0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0
    }

    if (msg.topleftX !== undefined) {
      resolved.topleftX = msg.topleftX;
    }
    else {
      resolved.topleftX = 0
    }

    if (msg.topleftY !== undefined) {
      resolved.topleftY = msg.topleftY;
    }
    else {
      resolved.topleftY = 0
    }

    if (msg.bottomrightX !== undefined) {
      resolved.bottomrightX = msg.bottomrightX;
    }
    else {
      resolved.bottomrightX = 0
    }

    if (msg.bottomrightY !== undefined) {
      resolved.bottomrightY = msg.bottomrightY;
    }
    else {
      resolved.bottomrightY = 0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0
    }

    if (msg.image_data !== undefined) {
      resolved.image_data = sensor_msgs.msg.Image.Resolve(msg.image_data)
    }
    else {
      resolved.image_data = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

module.exports = laser_electronic_result;
