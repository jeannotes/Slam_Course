
(cl:in-package :asdf)

(defsystem "chapter2_tutorials-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "chapter2_srv1" :depends-on ("_package_chapter2_srv1"))
    (:file "_package_chapter2_srv1" :depends-on ("_package"))
  ))