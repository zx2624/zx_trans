;; Auto-generated. Do not edit!


(when (boundp 'object_detector_msgs::laser_electronic_result)
  (if (not (find-package "OBJECT_DETECTOR_MSGS"))
    (make-package "OBJECT_DETECTOR_MSGS"))
  (shadow 'laser_electronic_result (find-package "OBJECT_DETECTOR_MSGS")))
(unless (find-package "OBJECT_DETECTOR_MSGS::LASER_ELECTRONIC_RESULT")
  (make-package "OBJECT_DETECTOR_MSGS::LASER_ELECTRONIC_RESULT"))

(in-package "ROS")
;;//! \htmlinclude laser_electronic_result.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_detector_msgs::laser_electronic_result
  :super ros::object
  :slots (_header _frameID _targetType _confidence _topleftX _topleftY _bottomrightX _bottomrightY _distance _image_data ))

(defmethod object_detector_msgs::laser_electronic_result
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:frameID __frameID) 0)
    ((:targetType __targetType) 0)
    ((:confidence __confidence) 0)
    ((:topleftX __topleftX) 0)
    ((:topleftY __topleftY) 0)
    ((:bottomrightX __bottomrightX) 0)
    ((:bottomrightY __bottomrightY) 0)
    ((:distance __distance) 0)
    ((:image_data __image_data) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _frameID (round __frameID))
   (setq _targetType (round __targetType))
   (setq _confidence (round __confidence))
   (setq _topleftX (round __topleftX))
   (setq _topleftY (round __topleftY))
   (setq _bottomrightX (round __bottomrightX))
   (setq _bottomrightY (round __bottomrightY))
   (setq _distance (round __distance))
   (setq _image_data __image_data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:frameID
   (&optional __frameID)
   (if __frameID (setq _frameID __frameID)) _frameID)
  (:targetType
   (&optional __targetType)
   (if __targetType (setq _targetType __targetType)) _targetType)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:topleftX
   (&optional __topleftX)
   (if __topleftX (setq _topleftX __topleftX)) _topleftX)
  (:topleftY
   (&optional __topleftY)
   (if __topleftY (setq _topleftY __topleftY)) _topleftY)
  (:bottomrightX
   (&optional __bottomrightX)
   (if __bottomrightX (setq _bottomrightX __bottomrightX)) _bottomrightX)
  (:bottomrightY
   (&optional __bottomrightY)
   (if __bottomrightY (setq _bottomrightY __bottomrightY)) _bottomrightY)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:image_data
   (&rest __image_data)
   (if (keywordp (car __image_data))
       (send* _image_data __image_data)
     (progn
       (if __image_data (setq _image_data (car __image_data)))
       _image_data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _frameID
    4
    ;; uint8 _targetType
    1
    ;; uint16 _confidence
    2
    ;; uint16 _topleftX
    2
    ;; uint16 _topleftY
    2
    ;; uint16 _bottomrightX
    2
    ;; uint16 _bottomrightY
    2
    ;; uint16 _distance
    2
    ;; sensor_msgs/Image _image_data
    (send _image_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _frameID
       (write-long _frameID s)
     ;; uint8 _targetType
       (write-byte _targetType s)
     ;; uint16 _confidence
       (write-word _confidence s)
     ;; uint16 _topleftX
       (write-word _topleftX s)
     ;; uint16 _topleftY
       (write-word _topleftY s)
     ;; uint16 _bottomrightX
       (write-word _bottomrightX s)
     ;; uint16 _bottomrightY
       (write-word _bottomrightY s)
     ;; uint16 _distance
       (write-word _distance s)
     ;; sensor_msgs/Image _image_data
       (send _image_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _frameID
     (setq _frameID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _targetType
     (setq _targetType (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _confidence
     (setq _confidence (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _topleftX
     (setq _topleftX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _topleftY
     (setq _topleftY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _bottomrightX
     (setq _bottomrightX (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _bottomrightY
     (setq _bottomrightY (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _distance
     (setq _distance (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; sensor_msgs/Image _image_data
     (send _image_data :deserialize buf ptr-) (incf ptr- (send _image_data :serialization-length))
   ;;
   self)
  )

(setf (get object_detector_msgs::laser_electronic_result :md5sum-) "28ce18d3994a9e82dc4a7dc982988a3b")
(setf (get object_detector_msgs::laser_electronic_result :datatype-) "object_detector_msgs/laser_electronic_result")
(setf (get object_detector_msgs::laser_electronic_result :definition-)
      "Header header
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

")



(provide :object_detector_msgs/laser_electronic_result "28ce18d3994a9e82dc4a7dc982988a3b")


