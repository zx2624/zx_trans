
(cl:in-package :asdf)

(defsystem "object_detector_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "laser_electronic_result" :depends-on ("_package_laser_electronic_result"))
    (:file "_package_laser_electronic_result" :depends-on ("_package"))
  ))