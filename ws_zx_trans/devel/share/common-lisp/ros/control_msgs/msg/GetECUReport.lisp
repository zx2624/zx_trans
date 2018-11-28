; Auto-generated. Do not edit!


(cl:in-package control_msgs-msg)


;//! \htmlinclude GetECUReport.msg.html

(cl:defclass <GetECUReport> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode
    :reader mode
    :initarg :mode
    :type control_msgs-msg:ModeReport
    :initform (cl:make-instance 'control_msgs-msg:ModeReport))
   (steer_cur
    :reader steer_cur
    :initarg :steer_cur
    :type control_msgs-msg:SteerReport
    :initform (cl:make-instance 'control_msgs-msg:SteerReport))
   (shift_cur
    :reader shift_cur
    :initarg :shift_cur
    :type control_msgs-msg:GearReport
    :initform (cl:make-instance 'control_msgs-msg:GearReport))
   (shift1_cur
    :reader shift1_cur
    :initarg :shift1_cur
    :type control_msgs-msg:GearReport
    :initform (cl:make-instance 'control_msgs-msg:GearReport))
   (brake_cur
    :reader brake_cur
    :initarg :brake_cur
    :type control_msgs-msg:BrakeReport
    :initform (cl:make-instance 'control_msgs-msg:BrakeReport))
   (throttle_cur
    :reader throttle_cur
    :initarg :throttle_cur
    :type control_msgs-msg:ThrottleReport
    :initform (cl:make-instance 'control_msgs-msg:ThrottleReport))
   (speed
    :reader speed
    :initarg :speed
    :type control_msgs-msg:SpeedReport
    :initform (cl:make-instance 'control_msgs-msg:SpeedReport))
   (manual
    :reader manual
    :initarg :manual
    :type control_msgs-msg:HMIReport
    :initform (cl:make-instance 'control_msgs-msg:HMIReport)))
)

(cl:defclass GetECUReport (<GetECUReport>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetECUReport>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetECUReport)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_msgs-msg:<GetECUReport> is deprecated: use control_msgs-msg:GetECUReport instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:header-val is deprecated.  Use control_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:mode-val is deprecated.  Use control_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'steer_cur-val :lambda-list '(m))
(cl:defmethod steer_cur-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:steer_cur-val is deprecated.  Use control_msgs-msg:steer_cur instead.")
  (steer_cur m))

(cl:ensure-generic-function 'shift_cur-val :lambda-list '(m))
(cl:defmethod shift_cur-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:shift_cur-val is deprecated.  Use control_msgs-msg:shift_cur instead.")
  (shift_cur m))

(cl:ensure-generic-function 'shift1_cur-val :lambda-list '(m))
(cl:defmethod shift1_cur-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:shift1_cur-val is deprecated.  Use control_msgs-msg:shift1_cur instead.")
  (shift1_cur m))

(cl:ensure-generic-function 'brake_cur-val :lambda-list '(m))
(cl:defmethod brake_cur-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:brake_cur-val is deprecated.  Use control_msgs-msg:brake_cur instead.")
  (brake_cur m))

(cl:ensure-generic-function 'throttle_cur-val :lambda-list '(m))
(cl:defmethod throttle_cur-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:throttle_cur-val is deprecated.  Use control_msgs-msg:throttle_cur instead.")
  (throttle_cur m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:speed-val is deprecated.  Use control_msgs-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'manual-val :lambda-list '(m))
(cl:defmethod manual-val ((m <GetECUReport>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_msgs-msg:manual-val is deprecated.  Use control_msgs-msg:manual instead.")
  (manual m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetECUReport>) ostream)
  "Serializes a message object of type '<GetECUReport>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'steer_cur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'shift_cur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'shift1_cur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'brake_cur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'throttle_cur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'speed) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'manual) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetECUReport>) istream)
  "Deserializes a message object of type '<GetECUReport>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'steer_cur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'shift_cur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'shift1_cur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'brake_cur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'throttle_cur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'speed) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'manual) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetECUReport>)))
  "Returns string type for a message object of type '<GetECUReport>"
  "control_msgs/GetECUReport")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetECUReport)))
  "Returns string type for a message object of type 'GetECUReport"
  "control_msgs/GetECUReport")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetECUReport>)))
  "Returns md5sum for a message object of type '<GetECUReport>"
  "83102858bdab6b0e68d7ca00281fe1a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetECUReport)))
  "Returns md5sum for a message object of type 'GetECUReport"
  "83102858bdab6b0e68d7ca00281fe1a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetECUReport>)))
  "Returns full string definition for message of type '<GetECUReport>"
  (cl:format cl:nil "Header header~%~%control_msgs/ModeReport mode~%~%control_msgs/SteerReport steer_cur~%~%control_msgs/GearReport shift_cur~%control_msgs/GearReport shift1_cur~%~%control_msgs/BrakeReport brake_cur~%control_msgs/ThrottleReport throttle_cur # include engine_status~%~%control_msgs/SpeedReport speed~%~%control_msgs/HMIReport manual~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: control_msgs/ModeReport~%Header header~%~%int32 auto_mode~%~%int32 speed_mode~%bool throttle_enable~%bool brake_enable~%~%int32 steer_mode~%int32 gear_mode~%~%~%================================================================================~%MSG: control_msgs/SteerReport~%Header header~%float32 steer~%~%================================================================================~%MSG: control_msgs/GearReport~%Header header~%uint8 gear~%~%================================================================================~%MSG: control_msgs/BrakeReport~%Header header~%~%float32 brake_ESC_left~%float32 brake_ESC_right~%~%control_msgs/WheelStateReport brake_wheel~%~%float32 brake_pedal~%~%================================================================================~%MSG: control_msgs/WheelStateReport~%Header header~%~%float32 front_left~%float32 front_right~%float32 rear_left~%float32 rear_right~%~%================================================================================~%MSG: control_msgs/ThrottleReport~%Header header~%float32 throttle~%control_msgs/EngineReport engine_status~%float32 throttle_pedal~%~%================================================================================~%MSG: control_msgs/EngineReport~%Header header~%float32 engine_rpm~%float32 engine_load~%~%================================================================================~%MSG: control_msgs/SpeedReport~%Header header~%~%control_msgs/WheelStateReport speed_wheel~%geometry_msgs/Twist velocity~%~%~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: control_msgs/HMIReport~%Header header~%~%bool manual_estop~%bool is_human_brake ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetECUReport)))
  "Returns full string definition for message of type 'GetECUReport"
  (cl:format cl:nil "Header header~%~%control_msgs/ModeReport mode~%~%control_msgs/SteerReport steer_cur~%~%control_msgs/GearReport shift_cur~%control_msgs/GearReport shift1_cur~%~%control_msgs/BrakeReport brake_cur~%control_msgs/ThrottleReport throttle_cur # include engine_status~%~%control_msgs/SpeedReport speed~%~%control_msgs/HMIReport manual~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: control_msgs/ModeReport~%Header header~%~%int32 auto_mode~%~%int32 speed_mode~%bool throttle_enable~%bool brake_enable~%~%int32 steer_mode~%int32 gear_mode~%~%~%================================================================================~%MSG: control_msgs/SteerReport~%Header header~%float32 steer~%~%================================================================================~%MSG: control_msgs/GearReport~%Header header~%uint8 gear~%~%================================================================================~%MSG: control_msgs/BrakeReport~%Header header~%~%float32 brake_ESC_left~%float32 brake_ESC_right~%~%control_msgs/WheelStateReport brake_wheel~%~%float32 brake_pedal~%~%================================================================================~%MSG: control_msgs/WheelStateReport~%Header header~%~%float32 front_left~%float32 front_right~%float32 rear_left~%float32 rear_right~%~%================================================================================~%MSG: control_msgs/ThrottleReport~%Header header~%float32 throttle~%control_msgs/EngineReport engine_status~%float32 throttle_pedal~%~%================================================================================~%MSG: control_msgs/EngineReport~%Header header~%float32 engine_rpm~%float32 engine_load~%~%================================================================================~%MSG: control_msgs/SpeedReport~%Header header~%~%control_msgs/WheelStateReport speed_wheel~%geometry_msgs/Twist velocity~%~%~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: control_msgs/HMIReport~%Header header~%~%bool manual_estop~%bool is_human_brake ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetECUReport>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'steer_cur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'shift_cur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'shift1_cur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'brake_cur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'throttle_cur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'speed))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'manual))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetECUReport>))
  "Converts a ROS message object to a list"
  (cl:list 'GetECUReport
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':steer_cur (steer_cur msg))
    (cl:cons ':shift_cur (shift_cur msg))
    (cl:cons ':shift1_cur (shift1_cur msg))
    (cl:cons ':brake_cur (brake_cur msg))
    (cl:cons ':throttle_cur (throttle_cur msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':manual (manual msg))
))
