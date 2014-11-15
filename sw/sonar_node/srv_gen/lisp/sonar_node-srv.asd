
(cl:in-package :asdf)

(defsystem "sonar_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sonar_node-msg
)
  :components ((:file "_package")
    (:file "FindPingers" :depends-on ("_package_FindPingers"))
    (:file "_package_FindPingers" :depends-on ("_package"))
  ))