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

(defun tasks-with-nearby-location (&optional (top-level-name :top-level)
                                     (action-type :transporting-from-container))
  (let* ((transporting-tasks
           (cut:force-ll
            (prolog:prolog `(task-specific-action ,top-level-name ((demo-fridge)) ,action-type ?task ?desig))))
        (match)
        (matching-pairs (list)))
    (loop while transporting-tasks
          do (when (setf match
                         (find-if (lambda (x) (location-desig-nearby
                                               (desig:desig-prop-value 
                                                (cut:var-value '?desig (car transporting-tasks)) :located-at)
                                               (desig:desig-prop-value 
                                                (cut:var-value '?desig x) :located-at)))
                                  (cdr transporting-tasks)))
               (push (list (car transporting-tasks) match) matching-pairs))
             (setf transporting-tasks (remove match (cdr transporting-tasks))))
    matching-pairs))

(defun tray-transporting-action ()
  (let* ((?tray-obj (desig:an object
                             (type :tray)))
        (?fetching-location
          (desig:a location
                   (on "CounterTop")
                   (name "iai_kitchen_sink_area_counter_top")))
        (?placing-target-pose
          (cl-transforms-stamped:pose->pose-stamped
           "map" 0.0
           (cram-bullet-reasoning:ensure-pose
            '((-0.75 1.85 0.85) (0 0 1 0)))))
        (?delivering-location
          (desig:a location
                   (pose ?placing-target-pose)))
        (action (desig:an action
                          (type transporting)
                          (object ?tray-obj)
                          (arm :right)
                          (location ?fetching-location)
                          (target ?delivering-location)
                          (retract-arms nil))))
    action))

(defun get-location-from-task (task)
  (desig:description
   (desig:desig-prop-value 
    (cut:var-value '?desig task) :location)))

(defun location-desig-nearby (desig-1 desig-2 &optional (threshold 0.2))
  (> threshold (cl-tf:v-dist (cl-tf:origin (desig-prop-value desig-1 :pose))
                             (cl-tf:origin (desig-prop-value desig-2 :pose)))))

(defun location-desig-dist (desig-1 desig-2)
  (cl-tf:v-dist (cl-tf:origin (desig-prop-value desig-1 :pose))
                (cl-tf:origin (desig-prop-value desig-2 :pose))))

(defun tasks-with-matching-location (&optional (path '((demo-random)))
                                       (top-level-name :top-level)
                                       (action-type :transporting))
  (let* ((transporting-tasks
           (cut:force-ll
            (prolog:prolog `(task-specific-action ,top-level-name ,path ,action-type ?task ?desig))))
        (match)
        (matching-pairs (list)))
    (loop while transporting-tasks
          do (when (setf match
                         (find-if (lambda (x) (equalp (get-location-from-task (car transporting-tasks))
                                                      (get-location-from-task x))) (cdr transporting-tasks)))
               (push (list (car transporting-tasks) match) matching-pairs))
             (setf transporting-tasks (remove match (cdr transporting-tasks))))
    matching-pairs))

(defun action-designator-under-path (path action-type &optional (top-level-name :top-level))
  (alexandria:assoc-value
   (car (cut:force-ll 
         (prolog:prolog `(and
                          (task-specific-action ,top-level-name ,path ,action-type ?task ?desig))))) '?desig))

(defun get-kitchen-urdf ()
  (slot-value
   (btr:object btr:*current-bullet-world* :kitchen)
   'cram-bullet-reasoning:urdf))

(defun move-kitchen-joint (&key (joint-name "iai_fridge_door_joint")
                             (joint-angle 2.5d0) (kitchen-name :kitchen))
  (btr:set-robot-state-from-joints
   `((,joint-name  ,joint-angle))
   (btr:object btr:*current-bullet-world* kitchen-name)))

(defun check-visibility (&optional (?type :cup))
  (pr2-proj:with-projected-robot
          (perform
           (an action
               (type detecting)
               (object (an object (type ?type)))))))

(defun spawn-cup-in-fridge ()
  (btr-utils:spawn-object :cup-1 :cup :pose '((1.35 0.6 0.9) (0 0 0 1)))
  (setf (btr::mass (car (btr:rigid-bodies (btr:object btr:*current-bullet-world* :cup-1)))) 0.0)
  (btr-utils:move-object :cup-1 (cdr (assoc :cup *object-spawning-in-container-poses*))))

(defun fetch-by-coord ()
  (pr2-proj:with-simulated-robot
        (let* ((?obj (an object
                        (type cup)))
              (?pose (cl-transforms-stamped:make-pose-stamped
                        "base_footprint" 0.0
                        (cl-transforms:make-3d-vector 1.3 -0.6 1.2)
                        (cl-transforms:make-identity-rotation)))
              (?loc (a location
                       (pose ?pose))))
        (exe:perform (desig:an action
                               (type fetching)
                               (object ?obj)
                               (location ?loc))))))

(defun move-just-the-gripper ()
  (pr2-proj:with-simulated-robot
        (let* ((?pose (cl-transforms-stamped:make-pose-stamped
                        "base_footprint" 0.0
                        (cl-transforms:make-3d-vector 0.6 0.2 1.2)
                        (cl-transforms:make-identity-rotation)))
               (?loc (a location
                        (pose ?pose))))
          (perform (a motion
                      (type moving-tcp)
                      (right-target ?loc))))))

(defun add-objects-to-mesh-list (&optional (ros-package "cram_bullet_world_tutorial"))
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (let ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                object-filename :keyword)))
                (pushnew (list lisp-name
                               (format nil "package://~a/resource/~a.~a"
                                       ros-package object-filename object-extension)
                               nil)
                         btr::*mesh-files*
                         :key #'car)
                lisp-name)))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/resource/*.*" ros-package))))))

;; (defun get-tltt (tltt-name)
;;   (cpl:get-top-level-task-tree tltt-name))

;; (defun available-arms ()
;;   (mapcar 'cdr
;;           (let ((link-list '(("l_wrist_roll_link" . :left) ("r_wrist_roll_link" . :right))))
;;             (set-difference link-list (mapcar (lambda (entry) (btr::attachment-link (caadr entry)))
;;                                               (btr:attached-objects (btr:get-robot-object)))
;;                             :test #'(lambda (a1 a2) (string= (car a1) a2))))))

;; (defun get-children (node)
;;        (if (null (cpl:task-tree-node-children node))
;;            (first (cpl:task-tree-node-path node))
;;            (append (list (first (cpl:task-tree-node-path node)))
;;                    (list (mapcar (lambda (child-par) (get-children (cdr child-par)))
;;                                  (cpl:task-tree-node-children node))))))

;; (defun find-task (task-name node)
;;   (let ((matches '()))
;;     (if (and node (equal task-name (first (cpl:task-tree-node-path node))))
;;              (setf matches (append matches (list node)))
;;              (setf matches (append matches (map 'list (lambda (child-par) (find-task task-name (cdr child-par)))
;;                                   (cpl:task-tree-node-children node)))))
;;     (alexandria:flatten matches)))

;; (defun get-robot-position (node)
;;   (first (cpl-impl:code-parameters (cpl:task-tree-node-code (cdr (find-if (lambda (entry) (equal (car entry) '(navigate-to)))
;;            (cpl:task-tree-node-children (cpl:task-tree-node-parent (cpl:task-tree-node-parent node)))))))))


;; (defun reset-fast ()
;;   (reset)
;;   (tt2)
;;   (reset nil)
;;   (apply-rules)
;;   (tt2))


;; (defun get-location-of-transport-action ()
;;   (desig:desig-prop-value 
;;    (cut:var-value '?desig 
;;                   (car (cut:force-ll 
;;                         (prolog:prolog
;;                          `(and
;;                            (task-specific-action :top-level ((demo-random)) 
;;                                                  :transporting ?task ?desig)))))) :location))

;; (defun test ()
;;   (cet:enable-fluent-tracing)
;;   (cpl-impl::remove-top-level-task-tree :top-level)

;;   (pr2-proj:with-simulated-robot
;;     (demo-random nil)))
