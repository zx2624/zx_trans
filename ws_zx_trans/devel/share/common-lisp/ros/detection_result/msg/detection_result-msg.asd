
(cl:in-package :asdf)

(defsystem "detection_result-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "detection_result_msg" :depends-on ("_package_detection_result_msg"))
    (:file "_package_detection_result_msg" :depends-on ("_package"))
  ))