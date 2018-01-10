(in-package :cpl)

(defvar *stop-infinity* 0)

(defun reset ()
  (setf *stop-infinity* 0)
  (cpl-impl::remove-top-level-task-tree 'ptr-fail-handle))

(def-cram-function failure-causing () 
  (if (> 1 *stop-infinity*) 
      (progn
        (setf *stop-infinity* (+ 1 *stop-infinity*))
        (format T "FAILURE-CAUSING: Will attempt to send a fail signal from ~A.~%" *current-path*) 
        (fail 'plan-failure)))
  (format T "FAILURE-CAUSING: Why are we back here?~%"))

(def-cram-function some-function ()
  (failure-causing))

(defun plan-repaired () 
  (format T "PLAN-REPAIRED: Hi there. This was inserted via code replacement.~%"))

(def-top-level-cram-function ptr-fail-handle () 
  (with-failure-handling
    ((plan-failure (f)
       (let ((code-path '((TOP-LEVEL PTR-FAIL-HANDLE))))
         (format T "Code path of failure: ~A.~% Current path: ~A.~%" code-path *current-path*)
         (cpl-impl::replace-task-code '(plan-repaired) #'plan-repaired code-path)
         (retry)
         )))
    (progn 
      (some-function))))
