(defsystem plan-transformation
  :depends-on (roslisp
               cram-language
               alexandria
               plan-transformation-tests)
  :components
  ((:module "lisp"
    :components
    ((:file "package")
     (:file "reader" :depends-on ("package"))))))
