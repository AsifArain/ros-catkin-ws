
(cl:in-package :asdf)

(defsystem "octo_raytrace-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "raytrace_rmld" :depends-on ("_package_raytrace_rmld"))
    (:file "_package_raytrace_rmld" :depends-on ("_package"))
  ))