
(cl:in-package :asdf)

(defsystem "rmld_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "rmld_msg" :depends-on ("_package_rmld_msg"))
    (:file "_package_rmld_msg" :depends-on ("_package"))
  ))