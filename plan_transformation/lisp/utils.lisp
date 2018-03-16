;;;
;;; Copyright (c) 2018, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                    
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

(defun get-top-level-name ()
  "For prolog predicates."
  *top-level-name*)

(defun  get-location-distance-threshold ()
  "For prolog predicates."
  *location-distance-threshold*)

(defparameter *object-cad-models*
  '((:cup . "cup_eco_orange")
    (:bowl . "edeka_red_bowl")))

(defun get-top-level-path (&optional (top-level-name *top-level-name*))
  (cpl:task-tree-node-path
   (cdr (car (direct-child
              (cpl:get-top-level-task-tree top-level-name))))))

(defgeneric direct-child (node)
  (:documentation "Returns only the direct children of the node")
  (:method ((node cpl:task-tree-node))
    (cpl:task-tree-node-children node)))

(defun flatten-task-tree-broad (task &optional (tree '()) (children '()))
  "Retruns the task tree as list of all tasks like `cpl:flatten-task-tree'
but sorted in broad first, not depth first."
  (if (and task (not tree) (not children))
      (flatten-task-tree-broad nil (list task) (list task))
      (let ((childs (mapcar #'cdr (reduce #'append (mapcar #'cpl:task-tree-node-children children)))))
        (if childs
            (flatten-task-tree-broad nil (append tree childs) childs)
            tree))))

(defun failed-tasks (top-task)
  (let ((predicate
          (lambda (node)
            (and (cpl:task-tree-node-code node)
                 (cpl:code-task (cpl:task-tree-node-code node))
                 (cpl:task-failed-p (cpl:code-task (cpl:task-tree-node-code node))))))
        (all-tasks (flatten-task-tree-broad top-task)))
    (loop for node in all-tasks
           when (funcall predicate node)
             collect node)))

(defun action-tasks-of-type (task type)
  (tasks-of-type task type 'action-designator))

(defun motion-tasks-of-type (task type)
  (tasks-of-type task type 'motion-designator))

(defun tasks-of-type (task type desig-class)
  (let* ((subtasks (case desig-class
                     (action-designator (flatten-task-tree-broad task))
                     (motion-designator (cpl:flatten-task-tree task))))
         (filter-predicate
           (lambda (node)
             (and (cpl:task-tree-node-code node)
                  (cpl:code-parameters
                   (cpl:task-tree-node-code node))
                  (cpl:code-sexp
                   (cpl:task-tree-node-code node))
                  (eq (car (cpl:code-sexp (cpl:task-tree-node-code node)))
                      'perform)
                  (eq (desig-prop-value
                       (car (cpl:code-parameters (cpl:task-tree-node-code node)))
                       :type)
                      type)
                  (eq (type-of
                       (car (cpl:code-parameters (cpl:task-tree-node-code node))))
                      desig-class)))))
    (remove-if-not filter-predicate subtasks)))

(defun reset-scene (&optional (top-level-name nil) (random nil))
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)
  (when (eql cram-projection:*projection-environment*
             'cram-pr2-projection::pr2-bullet-projection-environment)
    (if random
        (spawn-objects-on-sink-counter-randomly)
        (spawn-objects-on-sink-counter)))
  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 1)
  (initialize-or-finalize)
  (let ((robot-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "robot_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (cram-robot-interfaces:robot ?robot)
           (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
           (cram-robot-interfaces:robot-arms-parking-joint-states ?robot ?joint-states)
           (assert (btr:joint-state ?world ?robot ?joint-states))
           (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.22d0)))))))
  (when top-level-name
    (cpl-impl::remove-top-level-task-tree top-level-name)))

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
            (cdr (assoc :tray *object-placing-poses*)))))
        (?delivering-location
          (desig:a location
                   (pose ?placing-target-pose)))
        (action (desig:an action
                          (type transporting)
                          (object ?tray-obj)
                          (arm :right)
                          (location ?fetching-location)
                          (target ?delivering-location)
                          (retract-arms nil)))) action))

(defun search-tray ()
  (perform (an action
               (type searching)
               (object (an object (type :tray)))
               (location (a location
                            (on "CounterTop")
                            (name "iai_kitchen_sink_area_counter_top")
                            (side right))))))

(defun tray-pose (&optional (tray-obj (search-tray)))
  (let* ((rpos (btr:pose (btr:object btr:*current-bullet-world* 'cram-pr2-description:pr2)))
         (tpos (cadr (assoc :pose (desig-prop-value tray-obj :pose)))))
    (cl-tf:pose->pose-stamped
     "map" 0.0
     (cl-tf:transform (cl-tf:pose->transform rpos) tpos))))

(defun find-position-on-tray-for-item (item-type &optional (tray-pose nil))
  (let ((tray-pose (if tray-pose tray-pose (tray-pose))))
    (destructuring-bind
        ((x y z) (ax ay az aw))
        (alexandria:assoc-value *tray-pose-transforms* item-type)
      (cl-tf:make-pose-stamped
       "map" 0.0
       (cl-tf:origin
        (cl-tf:transform
         (cl-tf:make-transform (cl-tf:make-3d-vector x y z)
                               (cl-tf:make-identity-rotation))
         tray-pose))
       (cl-tf:make-quaternion ax ay az aw)))))

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
