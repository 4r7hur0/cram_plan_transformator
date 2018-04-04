;;;
;;; Copyright (c) 2018, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(defun test-collect-from-sink (&optional (reset t))
  (cet:enable-fluent-tracing)
  (when reset
    (cpl-impl::remove-top-level-task-tree *top-level-name*))
  (pr2-proj:with-simulated-robot
    (collect-from-sink nil))
  (cet:disable-fluent-tracing))

(defun test-collect-from-fridge (&optional (reset t))
  (cet:enable-fluent-tracing)
  (when reset
    (cpl-impl::remove-top-level-task-tree *top-level-name*))
  (pr2-proj:with-simulated-robot
    (collect-from-fridge))
  (cet:disable-fluent-tracing))

(defun test-collect-mixed (&optional (reset t))
  (cet:enable-fluent-tracing)
  (when reset
    (cpl-impl::remove-top-level-task-tree *top-level-name*))
  (pr2-proj:with-simulated-robot
    (collect-mixed))
  (cet:disable-fluent-tracing))

(cpl:def-cram-function initialize-or-finalize ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (pp-plans::park-arms)
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    cram-tf:*fixed-frame*
                    0.0
                    (cl-transforms:make-identity-vector)
                    (cl-transforms:make-identity-rotation))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location
                                    (pose ?pose))))))
      (exe:perform (desig:an action (type opening) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))


(cpl:def-cram-function collect-from-sink (&optional (random nil))
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)
  (when (eql cram-projection:*projection-environment*
             'cram-pr2-projection::pr2-bullet-projection-environment)
    (if random
        (spawn-objects-on-sink-counter-randomly)
        (spawn-objects-on-sink-counter)))
  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 1)
  (initialize-or-finalize)
  (let ((list-of-objects '(:spoon
                           :breakfast-cereal
                           :milk
                           :cup
                            ;; :bowl  :tray 
                           )))
    (dolist (?object-type list-of-objects)
      (let* ((?cad-model
               (cdr (assoc ?object-type *object-cad-models*)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (desig:when ?cad-model
                           (cad-model ?cad-model))))
             (?fetching-location
               (desig:a location
                        (on "CounterTop")
                        (name "iai_kitchen_sink_area_counter_top")
                        ;; (side right)
                        ))
             (?placing-target-pose
               (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (if (not (eq ?object-type :tray))
                    (cram-bullet-reasoning:ensure-pose
                     (cdr (assoc ?object-type *object-placing-poses*)))
                    (cram-bullet-reasoning:ensure-pose
                     (cdr (assoc ?object-type *object-placing-poses*))))))
             (?arm-to-use
               (cdr (assoc ?object-type *object-grasping-arms*)))
             (?delivering-location
               (desig:a location
                        (pose ?placing-target-pose))))

        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))
          (let* ((?isnt-tray (not (eq ?object-type :tray))))
            (exe:perform (desig:an action
                                   (type transporting)
                                   (object ?object-to-fetch)
                                   (arm ?arm-to-use)
                                   (location ?fetching-location)
                                   (target ?delivering-location)
                                   (retract-arms ?isnt-tray))))))))
  (initialize-or-finalize)
  cpl:*current-path*)

(cpl:def-cram-function collect-from-fridge ()
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)
  (spawn-objects-into-fridge)
  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 1)
  (initialize-or-finalize)
  (let ((list-of-objects '(:milk
                           :cup
                           :breakfast-cereal
                           ;; :bowl :tray :spoon
                           )))
    (dolist (?object-type list-of-objects)
      (let* ((?cad-model
               (cdr (assoc ?object-type *object-cad-models*)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (desig:when ?cad-model
                           (cad-model ?cad-model))))
             ;; (?fetching-location
             ;;   (desig:a location
             ;;            (on "CounterTop")
             ;;            (name "iai_kitchen_sink_area_counter_top")
             ;;            (side right)))
             (?container-fetch-pose
               (cl-transforms-stamped:make-pose-stamped
                "map" 0.0
                (cl-transforms:make-3d-vector 1.3 -0.7 1.0)
                (cl-transforms:make-identity-rotation)))
             (?fetching-container-location
               (desig:a location
                        (pose ?container-fetch-pose)))
             (?container-pose
               (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (cram-bullet-reasoning:ensure-pose
                 '((0.8 -0.8 0) (0 0 0 1)))))
             (?container-location
               (desig:a location
                        (pose ?container-pose)))
             (?placing-target-pose
               (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (if (not (eq ?object-type :tray))
                    (cram-bullet-reasoning:ensure-pose
                     (cdr (assoc ?object-type *object-placing-poses*)))
                    (cram-bullet-reasoning:ensure-pose
                     (cdr (assoc ?object-type *object-placing-poses*))))))
             ;; (?arm-to-use
             ;;   (cdr (assoc ?object-type *object-grasping-arms*)))
             (?delivering-location
               (desig:a location
                        (pose ?placing-target-pose))))

        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))
          (let* ((?not-tray (not (eq ?object-type :tray))))
            
            (exe:perform (desig:an action
                                   (type transporting-from-container)
                                   (object ?object-to-fetch)
                                   (located-at ?container-location)
                                   (arm right)
                                   (location ?fetching-container-location)
                                   (target ?delivering-location)
                                   (retract-arms ?not-tray))))))))
  (initialize-or-finalize)
  cpl:*current-path*)

(cpl:def-cram-function collect-mixed ()
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)
  (when (eql cram-projection:*projection-environment*
             'cram-pr2-projection::pr2-bullet-projection-environment)
    (spawn-objects-mixed))
  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 1)
  (initialize-or-finalize)
  (let ((list-of-sink-objects '(:spoon :breakfast-cereal))
        (list-of-fridge-objects '(:milk :cup)))
    (dolist (?object-type (append list-of-sink-objects
                                  list-of-fridge-objects))
      (let* ((?cad-model
               (cdr (assoc ?object-type *object-cad-models*)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (desig:when ?cad-model
                           (cad-model ?cad-model))))
             (?fetching-location
               (desig:a location
                        (on "CounterTop")
                        (name "iai_kitchen_sink_area_counter_top")))
             (?container-fetch-pose
               (cl-transforms-stamped:make-pose-stamped
                "map" 0.0
                (cl-transforms:make-3d-vector 1.3 -0.6 1.0)
                (cl-transforms:make-identity-rotation)))
             (?fetching-container-location
               (desig:a location
                        (pose ?container-fetch-pose)))
             (?container-pose
               (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (cram-bullet-reasoning:ensure-pose
                 '((0.8 -0.8 0) (0 0 0 1)))))
             (?container-location
               (desig:a location
                        (pose ?container-pose)))
             (?placing-target-pose
               (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (if (not (eq ?object-type :tray))
                    (cram-bullet-reasoning:ensure-pose
                     (cdr (assoc ?object-type *object-placing-poses*)))
                    (cram-bullet-reasoning:ensure-pose
                     (cdr (assoc ?object-type *object-placing-poses*))))))
             (?arm-to-use
               (cdr (assoc ?object-type *object-grasping-arms*)))
             (?delivering-location
               (desig:a location
                        (pose ?placing-target-pose)))
             (?isnt-tray (not (eq ?object-type :tray))))

        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))
          (if (member ?object-type list-of-sink-objects)
              (exe:perform (an action
                               (type transporting)
                               (object ?object-to-fetch)
                               (arm ?arm-to-use)
                               (location ?fetching-location)
                               (target ?delivering-location)
                               (retract-arms ?isnt-tray)))
              (when (member ?object-type list-of-fridge-objects)
                (exe:perform (an action
                                 (type transporting-from-container)
                                 (object ?object-to-fetch)
                                 (located-at ?container-location)
                                 (arm right)
                                 (location ?fetching-container-location)
                                 (target ?delivering-location)
                                 (retract-arms ?isnt-tray)))))))))
        (initialize-or-finalize)
        cpl:*current-path*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; OLD FUNCTIONS BELOW ;;;

;; (defun test (&optional (reset t))
;;   (cet:enable-fluent-tracing)
;;   (when reset
;;     (cpl-impl::remove-top-level-task-tree :top-level))

;;   (pr2-proj:with-simulated-robot
;;     (demo-random nil))
;;   (cet:disable-fluent-tracing)
;;   ;; (cut:force-ll (prolog:prolog `(task-navigating-action :top-level ((demo-random))
;;   ;;                                                       ?task ?designator)))
;;   )

;; (defun test-next-sibling-time ()
;;   (cet:enable-fluent-tracing)
;;   (cpl-impl::remove-top-level-task-tree :top-level)

;;   (pr2-proj:with-simulated-robot
;;     (demo-random nil))

;;   (cut:force-ll
;;    (prolog:prolog `(and (task-navigating-action :top-level ((demo-random)) ?task ?des)
;;                         (task-next-sibling :top-level ?task ?next-task)
;;                         (task-created-at :top-level ?task ?created)
;;                         (task-created-at :top-level ?next-task ?next-created)
;;                         (format "time: ~a   time next: ~a~%" ?created ?next-created)))))

;; (defun test-failed-actions ()
;;   (cet:enable-fluent-tracing)
;;   (cpl-impl::remove-top-level-task-tree :top-level)

;;   (pr2-proj:with-simulated-robot
;;     (demo-random))

;;   (cut:force-ll
;;    (prolog:prolog `(and (task-specific-action :top-level ((demo-random)) :fetching ?task ?desig)
;;                         (task-outcome ?task :failed)
;;                         (format "desig: ~a~%" ?desig)))))

;; (defun find-location-for-pick-up-using-occasions ()
;;   (cet:enable-fluent-tracing)
;;   (cpl-impl::remove-top-level-task-tree :top-level)

;;   (let ((found-location))
;;     (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
;;       (cpl-impl::named-top-level (:name :top-level)
;;         (demo-random nil))

;;       (let ((top-level-name :top-level))
;;         (setf found-location
;;               (cut:var-value
;;                '?pick-location
;;                (car (cut:force-ll
;;                 (prolog:prolog
;;                  `(and (task-fetching-action ,top-level-name ((demo-random)) ?fetching-task ?_)
;;                        (task-full-path ?fetching-task ?fetching-path)
;;                        (task-picking-up-action ,top-level-name ?fetching-path ?picking-up-task ?_)
;;                        (task-outcome ?picking-up-task :succeeded)
;;                        (task-started-at ,top-level-name ?picking-up-task ?picking-up-start)
;;                        (cram-robot-interfaces:robot ?robot)
;;                        (btr:timeline ?timeline)
;;                        (coe:holds ?timeline (cpoe:loc ?robot ?pick-location)
;;                                   (coe:at ?picking-up-start))))))))))
;;     found-location))

;; (defun find-location-for-pick-up (&optional (projection-run-count 5))
;;   (flet ((get-pick-up-location (top-level-name path)
;;            (let* ((bindings
;;                     (car
;;                      (prolog:prolog
;;                       `(and
;;                         (task-fetching-action ,top-level-name ,path
;;                                               ?fetching-task ?_)
;;                         (task-full-path ?fetching-task ?fetching-path)
;;                         (task-picking-up-action ,top-level-name ?fetching-path
;;                                                 ?picking-up-task ?picking-up-designator)
;;                         (task-outcome ?picking-up-task :succeeded)
;;                         (task-previous-action-sibling ,top-level-name ?fetching-path
;;                                                       ?picking-up-task
;;                                                       :navigating ?navigating-task)
;;                         (task-navigating-action ,top-level-name ?fetching-path ?navigating-task
;;                                                 ?navigating-designator)))))
;;                   (picking-action
;;                     (cut:var-value '?picking-up-designator bindings))
;;                   (picking-action-newest
;;                     (unless (cut:is-var picking-action)
;;                       (desig:newest-effective-designator picking-action)))
;;                   (picking-arm
;;                     (when picking-action-newest
;;                       (third (desig:reference picking-action-newest))))
;;                   (navigating-action
;;                     (cut:var-value '?navigating-designator bindings))
;;                   (navigating-action-newest
;;                     (unless (cut:is-var navigating-action)
;;                       (desig:newest-effective-designator navigating-action)))
;;                   (picking-location
;;                     (when navigating-action
;;                       (desig:newest-effective-designator
;;                        (desig:desig-prop-value navigating-action :location)))))
;;              (list picking-location picking-arm))))

;;     (cet:enable-fluent-tracing)
;;     (cpl-impl::remove-top-level-task-tree :top-level)

;;     (let (paths)
;;       (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
;;         (cpl-impl::named-top-level (:name :top-level)
;;           (dotimes (n projection-run-count)
;;             (push (demo-random nil) paths))))

;;       (mapcar (alexandria:curry #'get-pick-up-location :top-level)
;;               paths))))

;; (defun find-location-for-pick-up-with-successful-put-down (&optional (projection-run-count 5))
;;   (flet ((get-pick-up-location (top-level-name path)
;;            (let* ((bindings
;;                     (car
;;                      (prolog:prolog
;;                       `(and
;;                         (task-fetching-action ,top-level-name ,path
;;                                               ?fetching-task ?_)
;;                         (task-full-path ?fetching-task ?fetching-path)
;;                         (task-picking-up-action ,top-level-name ?fetching-path
;;                                                 ?picking-up-task ?picking-up-designator)
;;                         (task-outcome ?picking-up-task :succeeded)
;;                         (task-previous-action-sibling ,top-level-name ?fetching-path
;;                                                       ?picking-up-task
;;                                                       :navigating ?navigating-task)
;;                         (task-navigating-action ,top-level-name ?fetching-path ?navigating-task
;;                                                 ?navigating-designator)
;;                         (task-delivering-action ,top-level-name ,path
;;                                                 ?delivering-task ?_)
;;                         (task-outcome ?delivering-task :succeeded)))))
;;                   (picking-action
;;                     (cut:var-value '?picking-up-designator bindings))
;;                   (picking-action-newest
;;                     (unless (cut:is-var picking-action)
;;                       (desig:newest-effective-designator picking-action)))
;;                   (picking-arm
;;                     (when picking-action-newest
;;                       (third (desig:reference picking-action-newest))))
;;                   (navigating-action
;;                     (cut:var-value '?navigating-designator bindings))
;;                   (navigating-action-newest
;;                     (unless (cut:is-var navigating-action)
;;                       (desig:newest-effective-designator navigating-action)))
;;                   (picking-location
;;                     (when navigating-action-newest
;;                       (desig:newest-effective-designator
;;                        (desig:desig-prop-value navigating-action-newest :location)))))
;;              (list picking-location picking-arm))))

;;     (cet:enable-fluent-tracing)
;;     (cpl-impl::remove-top-level-task-tree :top-level)

;;     (let (paths)
;;       (proj:with-projection-environment pr2-proj:pr2-bullet-projection-environment
;;         (cpl-impl::named-top-level (:name :top-level)
;;           (dotimes (n projection-run-count)
;;             (push (demo-random nil) paths))))

;;       (mapcar (alexandria:curry #'get-pick-up-location :top-level)
;;               paths))))


;; (cpl:def-cram-function demo-random (&optional (random t))
;;   (demo-stacking random))

;; ;; (cpl:def-cram-function demo-random (&optional (random t))
;; ;;   (btr:detach-all-objects (btr:get-robot-object))
;; ;;   (btr-utils:kill-all-objects)
  
;; ;;   ;; (when (eql cram-projection:*projection-environment*
;; ;;   ;;            'cram-pr2-projection::pr2-bullet-projection-environment)
;; ;;   ;;   (if random
;; ;;   ;;       (spawn-objects-on-sink-counter-randomly)
;; ;;   ;;       (spawn-objects-on-sink-counter)))

;; ;;   (spawn-objects-into-fridge)
  
;; ;;   (setf cram-robot-pose-guassian-costmap::*orientation-samples* 1)

;; ;;   (initialize-or-finalize)

;; ;;   (let ((list-of-objects '(:milk
;; ;;                            ;:breakfast-cereal
;; ;;                            :cup
;; ;;                            ;; :bowl
;; ;;                            ;:tray
;; ;;                            ;; :spoon
;; ;;                            )))
;; ;;     (dolist (?object-type list-of-objects)
;; ;;       (let* ((?cad-model
;; ;;                (cdr (assoc ?object-type *object-cad-models*)))
;; ;;              (?object-to-fetch
;; ;;                (desig:an object
;; ;;                          (type ?object-type)
;; ;;                          (desig:when ?cad-model
;; ;;                            (cad-model ?cad-model))))
;; ;;              (?fetching-location
;; ;;                (desig:a location
;; ;;                         (on "CounterTop")
;; ;;                         (name "iai_kitchen_sink_area_counter_top")
;; ;;                         (side right)))
;; ;;              (?container-fetch-pose
;; ;;                (cl-transforms-stamped:make-pose-stamped
;; ;;                         "base_footprint" 0.0
;; ;;                         (cl-transforms:make-3d-vector 1.3 -0.6 1.0)
;; ;;                         (cl-transforms:make-identity-rotation)))
;; ;;              (?fetching-container-location
;; ;;                (desig:a location
;; ;;                         (pose ?container-fetch-pose)))
;; ;;              (?container-pose
;; ;;                (cl-transforms-stamped:pose->pose-stamped
;; ;;                     "map" 0.0
;; ;;                     (cram-bullet-reasoning:ensure-pose
;; ;;                      '((0.8 -0.8 0) (0 0 0 1)))))
;; ;;              (?container-location
;; ;;                (desig:a location
;; ;;                         (pose ?container-pose)))
;; ;;              (?placing-target-pose
;; ;;                (cl-transforms-stamped:pose->pose-stamped
;; ;;                 "map" 0.0
;; ;;                 (if (not (eq ?object-type :tray))
;; ;;                     (cram-bullet-reasoning:ensure-pose
;; ;;                      (cdr (assoc ?object-type *object-placing-poses*)))
;; ;;                     (cram-bullet-reasoning:ensure-pose
;; ;;                      (cdr (assoc ?object-type *object-placing-poses*))))))
;; ;;              (?arm-to-use
;; ;;                (cdr (assoc ?object-type *object-grasping-arms*)))
;; ;;              (?delivering-location
;; ;;                (desig:a location
;; ;;                         (pose ?placing-target-pose))))

;; ;;         (cpl:with-failure-handling
;; ;;             ((common-fail:high-level-failure (e)
;; ;;                (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
;; ;;                (return)))
;; ;;           (let* (;; (?navigation-goal *island-nav-goal*)
;; ;;                  ;; (?ptu-goal *look-goal*)
;; ;;                  ;; (?obj-name (make-symbol (format nil "~a-1" (string-trim "\:" (write-to-string ?object-type)))))
;; ;;                  (?isnt-tray (not (eq ?object-type :tray)))
;; ;;                  (transporting-action (desig:an action
;; ;;                                        (type transporting)
;; ;;                                        (object ?object-to-fetch)
;; ;;                                        (arm ?arm-to-use)
;; ;;                                        (location ?fetching-location)
;; ;;                                        (target ?delivering-location)
;; ;;                                        (retract-arms ?isnt-tray)))
;; ;;                  (fridge-navigation (desig:an action
;; ;;                                               (type navigating)
;; ;;                                               (location ?container-location))))
            
;; ;;             (exe:perform (desig:an action
;; ;;                                        (type transporting-from-container)
;; ;;                                        (object ?object-to-fetch)
;; ;;                                        (located-at ?container-location)
;; ;;                                        (arm right)
;; ;;                                        (location ?fetching-container-location)
;; ;;                                        (target ?delivering-location)
;; ;;                                        (retract-arms ?isnt-tray)))
            
;; ;;             ;; (if ?isnt-tray
;; ;;             ;;     (progn (btr::attach-item :MILK-1
;; ;;             ;;                              (btr:object btr:*current-bullet-world* :TRAY-1))
;; ;;             ;;            ;; (btr::attach-item :CUP-1
;; ;;             ;;            ;;                   (btr:object btr:*current-bullet-world* :TRAY-1))
;; ;;             ;;            )
;; ;;             ;;     (progn (btr::detach-item :MILK-1 (btr:object btr:*current-bullet-world* :TRAY-1))
;; ;;             ;;            ;; (btr::detach-item :CUP-1 (btr:object btr:*current-bullet-world* :TRAY-1))
;; ;;             ;;            ))

;; ;;                 ;; (progn
;; ;;                 ;;   (exe:perform (desig:an action
;; ;;                 ;;                          (type searching)
;; ;;                 ;;                          (object ?object-to-fetch)
;; ;;                 ;;                          (location ?fetching-location)))
;; ;;                 ;;   (exe:perform (desig:an action
;; ;;                 ;;                          (type fetching)
;; ;;                 ;;                          (object ?object-to-fetch)
;; ;;                 ;;                          (location ?fetching-location)
;; ;;                 ;;                          (retract-arms nil)))
;; ;;                 ;;   (let* ((?pose-at-target-location (desig:reference ?delivering-location))
;; ;;                 ;;          (?nav-location (desig:a location
;; ;;                 ;;                                  (reachable-for pr2)
;; ;;                 ;;                                  (location (desig:a location
;; ;;                 ;;                                                     (pose ?pose-at-target-location))))))
;; ;;                 ;;     (exe:perform (desig:an action
;; ;;                 ;;                            (type navigating)
;; ;;                 ;;                            (location ?nav-location)
;; ;;                 ;;                            (retract-arms nil))))
;; ;;                 ;;   (btr::detach-item :BOWL-1 (btr:object btr:*current-bullet-world* :TRAY-1)))
                
                                   
;; ;;             ;; (exe:perform (desig:an action
;; ;;             ;;                        (type navigating)
;; ;;             ;;                        (location ?delivering-location)
;; ;;             ;;                        (retract-arms nil)))
;; ;;             ;; (exe:perform (desig:an action
;; ;;             ;;                       (type navigating)
;; ;;             ;;                       (location ?fetching-location)))
;; ;;             ;; (exe:perform (desig:a motion
;; ;;             ;;                       (type looking)
;; ;;             ;;                       (target (desig:a location (pose ?ptu-goal)))))
;; ;;             ;; (exe:perform (desig:an action
;; ;;             ;;                        (type placing)
;; ;;             ;;                        (arm ?arm-to-use)
;; ;;             ;;                        (target (desig:a location
;; ;;             ;;                                         (pose ?delivering-location)))))
;; ;;            ;; (exe:perform (desig:an action
;; ;;            ;;                        (type transporting)
;; ;;            ;;                        (object ?object-to-fetch)
;; ;;            ;;                        (arm ?arm-to-use)
;; ;;            ;;                        (location ?fetching-location)
;; ;;            ;;                        (target ?delivering-location)
;; ;;            ;;                        (retract-arms nil)))
;; ;;             ;; (setf bowl-tf (cl-transforms:transform-diff (cl-transforms:pose->transform (btr:object-pose :tray-1))
;; ;;             ;;                                             tray-pre-tf))
;; ;;             ;; (setf new-bowl-pose (cl-transforms:transform-pose bowl-tf (btr:object-pose :bowl-1)))
;; ;;             ;; (btr-utils:move-object :bowl-1 (pose->btr-pose new-bowl-pose))
;; ;;             )))))

;; ;;   (initialize-or-finalize)
;; ;;   cpl:*current-path*)



