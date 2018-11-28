;; Auto-generated. Do not edit!


(when (boundp 'control_msgs::GetECUReport)
  (if (not (find-package "CONTROL_MSGS"))
    (make-package "CONTROL_MSGS"))
  (shadow 'GetECUReport (find-package "CONTROL_MSGS")))
(unless (find-package "CONTROL_MSGS::GETECUREPORT")
  (make-package "CONTROL_MSGS::GETECUREPORT"))

(in-package "ROS")
;;//! \htmlinclude GetECUReport.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_msgs::GetECUReport
  :super ros::object
  :slots (_header _mode _steer_cur _shift_cur _shift1_cur _brake_cur _throttle_cur _speed _manual ))

(defmethod control_msgs::GetECUReport
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode __mode) (instance control_msgs::ModeReport :init))
    ((:steer_cur __steer_cur) (instance control_msgs::SteerReport :init))
    ((:shift_cur __shift_cur) (instance control_msgs::GearReport :init))
    ((:shift1_cur __shift1_cur) (instance control_msgs::GearReport :init))
    ((:brake_cur __brake_cur) (instance control_msgs::BrakeReport :init))
    ((:throttle_cur __throttle_cur) (instance control_msgs::ThrottleReport :init))
    ((:speed __speed) (instance control_msgs::SpeedReport :init))
    ((:manual __manual) (instance control_msgs::HMIReport :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode __mode)
   (setq _steer_cur __steer_cur)
   (setq _shift_cur __shift_cur)
   (setq _shift1_cur __shift1_cur)
   (setq _brake_cur __brake_cur)
   (setq _throttle_cur __throttle_cur)
   (setq _speed __speed)
   (setq _manual __manual)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mode
   (&rest __mode)
   (if (keywordp (car __mode))
       (send* _mode __mode)
     (progn
       (if __mode (setq _mode (car __mode)))
       _mode)))
  (:steer_cur
   (&rest __steer_cur)
   (if (keywordp (car __steer_cur))
       (send* _steer_cur __steer_cur)
     (progn
       (if __steer_cur (setq _steer_cur (car __steer_cur)))
       _steer_cur)))
  (:shift_cur
   (&rest __shift_cur)
   (if (keywordp (car __shift_cur))
       (send* _shift_cur __shift_cur)
     (progn
       (if __shift_cur (setq _shift_cur (car __shift_cur)))
       _shift_cur)))
  (:shift1_cur
   (&rest __shift1_cur)
   (if (keywordp (car __shift1_cur))
       (send* _shift1_cur __shift1_cur)
     (progn
       (if __shift1_cur (setq _shift1_cur (car __shift1_cur)))
       _shift1_cur)))
  (:brake_cur
   (&rest __brake_cur)
   (if (keywordp (car __brake_cur))
       (send* _brake_cur __brake_cur)
     (progn
       (if __brake_cur (setq _brake_cur (car __brake_cur)))
       _brake_cur)))
  (:throttle_cur
   (&rest __throttle_cur)
   (if (keywordp (car __throttle_cur))
       (send* _throttle_cur __throttle_cur)
     (progn
       (if __throttle_cur (setq _throttle_cur (car __throttle_cur)))
       _throttle_cur)))
  (:speed
   (&rest __speed)
   (if (keywordp (car __speed))
       (send* _speed __speed)
     (progn
       (if __speed (setq _speed (car __speed)))
       _speed)))
  (:manual
   (&rest __manual)
   (if (keywordp (car __manual))
       (send* _manual __manual)
     (progn
       (if __manual (setq _manual (car __manual)))
       _manual)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; control_msgs/ModeReport _mode
    (send _mode :serialization-length)
    ;; control_msgs/SteerReport _steer_cur
    (send _steer_cur :serialization-length)
    ;; control_msgs/GearReport _shift_cur
    (send _shift_cur :serialization-length)
    ;; control_msgs/GearReport _shift1_cur
    (send _shift1_cur :serialization-length)
    ;; control_msgs/BrakeReport _brake_cur
    (send _brake_cur :serialization-length)
    ;; control_msgs/ThrottleReport _throttle_cur
    (send _throttle_cur :serialization-length)
    ;; control_msgs/SpeedReport _speed
    (send _speed :serialization-length)
    ;; control_msgs/HMIReport _manual
    (send _manual :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; control_msgs/ModeReport _mode
       (send _mode :serialize s)
     ;; control_msgs/SteerReport _steer_cur
       (send _steer_cur :serialize s)
     ;; control_msgs/GearReport _shift_cur
       (send _shift_cur :serialize s)
     ;; control_msgs/GearReport _shift1_cur
       (send _shift1_cur :serialize s)
     ;; control_msgs/BrakeReport _brake_cur
       (send _brake_cur :serialize s)
     ;; control_msgs/ThrottleReport _throttle_cur
       (send _throttle_cur :serialize s)
     ;; control_msgs/SpeedReport _speed
       (send _speed :serialize s)
     ;; control_msgs/HMIReport _manual
       (send _manual :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; control_msgs/ModeReport _mode
     (send _mode :deserialize buf ptr-) (incf ptr- (send _mode :serialization-length))
   ;; control_msgs/SteerReport _steer_cur
     (send _steer_cur :deserialize buf ptr-) (incf ptr- (send _steer_cur :serialization-length))
   ;; control_msgs/GearReport _shift_cur
     (send _shift_cur :deserialize buf ptr-) (incf ptr- (send _shift_cur :serialization-length))
   ;; control_msgs/GearReport _shift1_cur
     (send _shift1_cur :deserialize buf ptr-) (incf ptr- (send _shift1_cur :serialization-length))
   ;; control_msgs/BrakeReport _brake_cur
     (send _brake_cur :deserialize buf ptr-) (incf ptr- (send _brake_cur :serialization-length))
   ;; control_msgs/ThrottleReport _throttle_cur
     (send _throttle_cur :deserialize buf ptr-) (incf ptr- (send _throttle_cur :serialization-length))
   ;; control_msgs/SpeedReport _speed
     (send _speed :deserialize buf ptr-) (incf ptr- (send _speed :serialization-length))
   ;; control_msgs/HMIReport _manual
     (send _manual :deserialize buf ptr-) (incf ptr- (send _manual :serialization-length))
   ;;
   self)
  )

(setf (get control_msgs::GetECUReport :md5sum-) "83102858bdab6b0e68d7ca00281fe1a4")
(setf (get control_msgs::GetECUReport :datatype-) "control_msgs/GetECUReport")
(setf (get control_msgs::GetECUReport :definition-)
      "Header header

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

")



(provide :control_msgs/GetECUReport "83102858bdab6b0e68d7ca00281fe1a4")


