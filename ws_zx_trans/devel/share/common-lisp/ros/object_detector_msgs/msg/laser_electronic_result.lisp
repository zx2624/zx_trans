; Auto-generated. Do not edit!


(cl:in-package object_detector_msgs-msg)


;//! \htmlinclude laser_electronic_result.msg.html

(cl:defclass <laser_electronic_result> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (frameID
    :reader frameID
    :initarg :frameID
    :type cl:integer
    :initform 0)
   (targetType
    :reader targetType
    :initarg :targetType
    :type cl:fixnum
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:fixnum
    :initform 0)
   (topleftX
    :reader topleftX
    :initarg :topleftX
    :type cl:fixnum
    :initform 0)
   (topleftY
    :reader topleftY
    :initarg :topleftY
    :type cl:fixnum
    :initform 0)
   (bottomrightX
    :reader bottomrightX
    :initarg :bottomrightX
    :type cl:fixnum
    :initform 0)
   (bottomrightY
    :reader bottomrightY
    :initarg :bottomrightY
    :type cl:fixnum
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:fixnum
    :initform 0)
   (image_data
    :reader image_data
    :initarg :image_data
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass laser_electronic_result (<laser_electronic_result>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <laser_electronic_result>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'laser_electronic_result)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detector_msgs-msg:<laser_electronic_result> is deprecated: use object_detector_msgs-msg:laser_electronic_result instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:header-val is deprecated.  Use object_detector_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'frameID-val :lambda-list '(m))
(cl:defmethod frameID-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:frameID-val is deprecated.  Use object_detector_msgs-msg:frameID instead.")
  (frameID m))

(cl:ensure-generic-function 'targetType-val :lambda-list '(m))
(cl:defmethod targetType-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:targetType-val is deprecated.  Use object_detector_msgs-msg:targetType instead.")
  (targetType m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:confidence-val is deprecated.  Use object_detector_msgs-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'topleftX-val :lambda-list '(m))
(cl:defmethod topleftX-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:topleftX-val is deprecated.  Use object_detector_msgs-msg:topleftX instead.")
  (topleftX m))

(cl:ensure-generic-function 'topleftY-val :lambda-list '(m))
(cl:defmethod topleftY-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:topleftY-val is deprecated.  Use object_detector_msgs-msg:topleftY instead.")
  (topleftY m))

(cl:ensure-generic-function 'bottomrightX-val :lambda-list '(m))
(cl:defmethod bottomrightX-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:bottomrightX-val is deprecated.  Use object_detector_msgs-msg:bottomrightX instead.")
  (bottomrightX m))

(cl:ensure-generic-function 'bottomrightY-val :lambda-list '(m))
(cl:defmethod bottomrightY-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:bottomrightY-val is deprecated.  Use object_detector_msgs-msg:bottomrightY instead.")
  (bottomrightY m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:distance-val is deprecated.  Use object_detector_msgs-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'image_data-val :lambda-list '(m))
(cl:defmethod image_data-val ((m <laser_electronic_result>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector_msgs-msg:image_data-val is deprecated.  Use object_detector_msgs-msg:image_data instead.")
  (image_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <laser_electronic_result>) ostream)
  "Serializes a message object of type '<laser_electronic_result>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'targetType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'confidence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'topleftX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'topleftX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'topleftY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'topleftY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottomrightX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottomrightX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottomrightY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottomrightY)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'distance)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'distance)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <laser_electronic_result>) istream)
  "Deserializes a message object of type '<laser_electronic_result>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'targetType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confidence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'confidence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'topleftX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'topleftX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'topleftY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'topleftY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottomrightX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottomrightX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottomrightY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottomrightY)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'distance)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'distance)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<laser_electronic_result>)))
  "Returns string type for a message object of type '<laser_electronic_result>"
  "object_detector_msgs/laser_electronic_result")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'laser_electronic_result)))
  "Returns string type for a message object of type 'laser_electronic_result"
  "object_detector_msgs/laser_electronic_result")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<laser_electronic_result>)))
  "Returns md5sum for a message object of type '<laser_electronic_result>"
  "28ce18d3994a9e82dc4a7dc982988a3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'laser_electronic_result)))
  "Returns md5sum for a message object of type 'laser_electronic_result"
  "28ce18d3994a9e82dc4a7dc982988a3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<laser_electronic_result>)))
  "Returns full string definition for message of type '<laser_electronic_result>"
  (cl:format cl:nil "Header header~%uint32 frameID~%uint8  targetType~%uint16 confidence~%uint16 topleftX~%uint16 topleftY~%uint16 bottomrightX~%uint16 bottomrightY~%uint16 distance~%sensor_msgs/Image  image_data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'laser_electronic_result)))
  "Returns full string definition for message of type 'laser_electronic_result"
  (cl:format cl:nil "Header header~%uint32 frameID~%uint8  targetType~%uint16 confidence~%uint16 topleftX~%uint16 topleftY~%uint16 bottomrightX~%uint16 bottomrightY~%uint16 distance~%sensor_msgs/Image  image_data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <laser_electronic_result>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     2
     2
     2
     2
     2
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <laser_electronic_result>))
  "Converts a ROS message object to a list"
  (cl:list 'laser_electronic_result
    (cl:cons ':header (header msg))
    (cl:cons ':frameID (frameID msg))
    (cl:cons ':targetType (targetType msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':topleftX (topleftX msg))
    (cl:cons ':topleftY (topleftY msg))
    (cl:cons ':bottomrightX (bottomrightX msg))
    (cl:cons ':bottomrightY (bottomrightY msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':image_data (image_data msg))
))
