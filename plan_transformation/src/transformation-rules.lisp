;;;
;;; Copyright (c) 2017, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :plt)

;; (register-transformation-rule both-hands-transporting-rule
;;                               '(task-transporting-siblings ?first-transport ?second-transport))
;; (register-transformation-rule stacking-rule
;;                               '(task-transporting-with-tray ?last-path))
;; (register-transformation-rule environment-rule
;;                               '(task-transporting-from-fridge ?action ?open-path ?close-path))

(defun both-hands-rule (lazy-bindings &optional (top-level-name (get-top-level-name)))
  (roslisp:ros-info (plt) "Applying BOTH-HANDS-RULE to top-level-plan ~a." top-level-name)
  (destructuring-bind
      ((key path-1) (other-key path-2 deliver-action))
      (cut:lazy-car lazy-bindings)
    (declare (ignore key other-key))
    (cpl-impl::replace-task-code '(BOTH-HANDS-TRANSFORM-1)
                                 #'(lambda (&rest desig)
                                     (declare (ignore desig))
                                     t)
                                 path-1
                                 (cpl-impl::get-top-level-task-tree top-level-name))
    (cpl-impl::replace-task-code '(BOTH-HANDS-TRANSFORM-2)
                                 #'(lambda (&rest desig)
                                     (exe:perform (car desig))
                                     (exe:perform deliver-action))
                                 path-2
                                 (cpl-impl::get-top-level-task-tree top-level-name))))

(defun tray-rule (lazy-bindings &optional (top-level-name (get-top-level-name)))
  (roslisp:ros-info (plt) "Applying TRAY-RULE to top-level-plan ~a." top-level-name)
  (let* ((paths (remove-duplicates (mapcar #'cadar (cut:force-ll (cut:lazy-take 4 lazy-bindings)))
                                            :test #'string= :key #'write-to-string))
         (last-path (pop paths))
         (rest-paths (reverse paths)))
    

    (labels ((change-loc-to-tray (action-desig)
               (let* ((obj-name
                        (desig-prop-value (desig-prop-value action-desig :object) :name))
                      (tray-transform-stamped (search-tray))
                      (?new-pose (pose-on-tray tray-transform-stamped obj-name))
                      (?desig-cpy (desig:description (desig:copy-designator action-desig))))
               ;; (setf tray-name (desig-prop-value tray-obj :name))
                 (setf ?desig-cpy (remove (assoc :target ?desig-cpy) ?desig-cpy))
                 (push `(:target ,(desig:a location
                                           (pose ?new-pose))) ?desig-cpy)
                 (desig:make-designator :action ?desig-cpy))))
      
      (loop for path in rest-paths
            counting t into index
            do (cpl-impl::replace-task-code
                `(,(intern (format nil "TRAY-TRANSFORM-~a" index)))
                #'(lambda (&rest desig)
                    (exe:perform (change-loc-to-tray (car desig)))
                    ;; (btr::attach-item (desig-prop-value (desig-prop-value (car desig) :object) :name)
                    ;;                   (btr-obj tray-name))
                    )
                path
                (cpl-impl::get-top-level-task-tree top-level-name)))

      (cpl-impl::replace-task-code
       '(STACKING-TRANSFORM-LAST)
       #'(lambda (&rest desig)
           (exe:perform (change-loc-to-tray (car desig)))
           ;; (btr::attach-item (desig-prop-value (desig-prop-value (car desig) :object) :name)
           ;;                   (btr-obj tray-name))
           (exe:perform (tray-transporting-action))
           ;; (btr::detach-all-items (btr-obj tray-name))
           )
       last-path
       (cpl-impl::get-top-level-task-tree top-level-name))
      )
    ))


(defun container-rule (lazy-bindings &optional (top-level-name (get-top-level-name)))
  (roslisp:ros-info (plt) "Applying CONTAINER-RULE to top-level-plan ~a." top-level-name)

  ;; TODO
  (let ((sealing-path (cdadr (cut:lazy-car lazy-bindings)))
        (accessing-path (cdar (cut:lazy-car lazy-bindings))))
    (cpl-impl::replace-task-code '(SEALING-TRANSFORM)
                                 #'(lambda (&rest desig)
                                     (declare (ignore desig))
                                     (roslisp:ros-info (sealing-transform)
                                                       "Sealing action deleted in transformation."))
                                 sealing-path
                                 (cpl-impl::get-top-level-task-tree top-level-name))
    (cpl-impl::replace-task-code '(ACCESSING-TRANSFORM)
                                 #'(lambda (&rest desig)
                                     (declare (ignore desig))
                                     (roslisp:ros-info (accessing-transform)
                                                       "Accessing action deleted in transformation."))
                                 accessing-path
                                 (cpl-impl::get-top-level-task-tree top-level-name)))
  
  ;; (let* ((bindings (remove-duplicates (cut:force-ll lazy-bindings)
  ;;                                     :test #'string= :key #'write-to-string))
  ;;        (last-action (pop bindings))
  ;;        (bindings (reverse bindings))
  ;;        (first-action (pop bindings)))
  ;;   (flet ((close-nothing (&rest desig) 
  ;;            (declare (ignore desig))
  ;;            (setf pr2-proj-reasoning::*allow-pick-up-kitchen-collision* NIL))
  ;;          (open-nothing (navigation-action) 
  ;;            (exe:perform navigation-action)
  ;;            (setf pr2-proj-reasoning::*allow-pick-up-kitchen-collision* T)))

  ;;   (destructuring-bind (x y closing-path) first-action
  ;;     (declare (ignore x y))
  ;;     (cpl-impl::replace-task-code '(CONTAINER-FIRST-CLOSING-TRANSFORM)
  ;;                                  #'close-nothing
  ;;                                  (cdr closing-path)
  ;;                                  (cpl-impl::get-top-level-task-tree top-level-name)))

  ;;   (destructuring-bind (navigation-action opening-path x) last-action
  ;;     (declare (ignore x))
  ;;     (cpl-impl::replace-task-code '(CONTAINER-LAST-OPENING-TRANSFORM)
  ;;                                  #'(lambda (&rest desig)
  ;;                                      (declare (ignore desig))
  ;;                                      (open-nothing (cdr navigation-action)))
  ;;                                  (cdr opening-path)
  ;;                                  (cpl-impl::get-top-level-task-tree top-level-name)))

  ;;     (loop for (navigation-action opening-path closing-path) in bindings
  ;;           counting t into index
  ;;           do (cpl-impl::replace-task-code `(,(intern (format nil "CONTAINER-ACCESS-TRANSFORM-~a" index)))
  ;;                                           #'close-nothing
  ;;                                           (cdr closing-path)
  ;;                                           (cpl-impl::get-top-level-task-tree top-level-name))
  ;;              (cpl-impl::replace-task-code `(,(intern (format nil "CONTAINER-CLOSE-TRANSFORM-~a" index)))
  ;;                                           #'(lambda (&rest desig)
  ;;                                               (declare (ignore desig))
  ;;                                               (open-nothing (cdr navigation-action)))
  ;;                                           (cdr opening-path)
  ;;                                           (cpl-impl::get-top-level-task-tree top-level-name)))))
  )
