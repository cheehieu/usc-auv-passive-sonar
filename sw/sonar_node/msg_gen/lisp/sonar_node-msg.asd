
(cl:in-package :asdf)

(defsystem "sonar_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Pinger" :depends-on ("_package_Pinger"))
    (:file "_package_Pinger" :depends-on ("_package"))
    (:file "PingerArray" :depends-on ("_package_PingerArray"))
    (:file "_package_PingerArray" :depends-on ("_package"))
  ))