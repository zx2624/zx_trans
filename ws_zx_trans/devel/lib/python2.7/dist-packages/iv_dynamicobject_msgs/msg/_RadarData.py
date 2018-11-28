# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from iv_dynamicobject_msgs/RadarData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import iv_dynamicobject_msgs.msg
import std_msgs.msg

class RadarData(genpy.Message):
  _md5sum = "11cc650889f4145ca7db11cb95c6e9cb"
  _type = "iv_dynamicobject_msgs/RadarData"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
RadarPoint[64] delphi_detection_array
uint8 ACC_Target_ID

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
MSG: iv_dynamicobject_msgs/RadarPoint
uint8 target_ID
float32 range
float32 v
float32 angle
float32 x
float32 y
bool valid
uint8 status
uint8 moving
bool moving_fast
bool moving_slow
"""
  __slots__ = ['header','delphi_detection_array','ACC_Target_ID']
  _slot_types = ['std_msgs/Header','iv_dynamicobject_msgs/RadarPoint[64]','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,delphi_detection_array,ACC_Target_ID

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RadarData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.delphi_detection_array is None:
        self.delphi_detection_array = [iv_dynamicobject_msgs.msg.RadarPoint() for _ in range(64)]
      if self.ACC_Target_ID is None:
        self.ACC_Target_ID = 0
    else:
      self.header = std_msgs.msg.Header()
      self.delphi_detection_array = [iv_dynamicobject_msgs.msg.RadarPoint() for _ in range(64)]
      self.ACC_Target_ID = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      for val1 in self.delphi_detection_array:
        _x = val1
        buff.write(_get_struct_B5f5B().pack(_x.target_ID, _x.range, _x.v, _x.angle, _x.x, _x.y, _x.valid, _x.status, _x.moving, _x.moving_fast, _x.moving_slow))
      buff.write(_get_struct_B().pack(self.ACC_Target_ID))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.delphi_detection_array is None:
        self.delphi_detection_array = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      self.delphi_detection_array = []
      for i in range(0, 64):
        val1 = iv_dynamicobject_msgs.msg.RadarPoint()
        _x = val1
        start = end
        end += 26
        (_x.target_ID, _x.range, _x.v, _x.angle, _x.x, _x.y, _x.valid, _x.status, _x.moving, _x.moving_fast, _x.moving_slow,) = _get_struct_B5f5B().unpack(str[start:end])
        val1.valid = bool(val1.valid)
        val1.moving_fast = bool(val1.moving_fast)
        val1.moving_slow = bool(val1.moving_slow)
        self.delphi_detection_array.append(val1)
      start = end
      end += 1
      (self.ACC_Target_ID,) = _get_struct_B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      for val1 in self.delphi_detection_array:
        _x = val1
        buff.write(_get_struct_B5f5B().pack(_x.target_ID, _x.range, _x.v, _x.angle, _x.x, _x.y, _x.valid, _x.status, _x.moving, _x.moving_fast, _x.moving_slow))
      buff.write(_get_struct_B().pack(self.ACC_Target_ID))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.delphi_detection_array is None:
        self.delphi_detection_array = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      self.delphi_detection_array = []
      for i in range(0, 64):
        val1 = iv_dynamicobject_msgs.msg.RadarPoint()
        _x = val1
        start = end
        end += 26
        (_x.target_ID, _x.range, _x.v, _x.angle, _x.x, _x.y, _x.valid, _x.status, _x.moving, _x.moving_fast, _x.moving_slow,) = _get_struct_B5f5B().unpack(str[start:end])
        val1.valid = bool(val1.valid)
        val1.moving_fast = bool(val1.moving_fast)
        val1.moving_slow = bool(val1.moving_slow)
        self.delphi_detection_array.append(val1)
      start = end
      end += 1
      (self.ACC_Target_ID,) = _get_struct_B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_B5f5B = None
def _get_struct_B5f5B():
    global _struct_B5f5B
    if _struct_B5f5B is None:
        _struct_B5f5B = struct.Struct("<B5f5B")
    return _struct_B5f5B