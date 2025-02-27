
(cl:in-package :asdf)

(defsystem "move-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "detect_state" :depends-on ("_package_detect_state"))
    (:file "_package_detect_state" :depends-on ("_package"))
  ))