(defsystem plan-transformation-tests
  :depends-on (roslisp
               cram-language
               alexandria)
  :components
  ((:module "lisp"
            :components
            ((:file "package")
             (:file "data-generator" :depends-on ("package"))
             (:file "utils" :depends-on ("package"))))))
