-;;;
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

(defparameter *top-level-name* :top-level)

(defun get-top-level-name ()
  "For prolog predicates."
  *top-level-name*)

(defgeneric direct-child (node)
  (:documentation "Returns only the direct children of the node")
  (:method ((node cpl:task-tree-node))
    (cpl:task-tree-node-children node)))

(defun get-top-level-path (&optional (top-level-name *top-level-name*))
  (cpl:task-tree-node-path
   (cdr (car (direct-child
              (cpl:get-top-level-task-tree top-level-name))))))

(defun flatten-task-tree-broad (task &optional (tree '()) (children '()))
  "Retruns the task tree as list of all tasks like `cpl:flatten-task-tree'
but sorted in broad first, not depth first."
  (if (and task (not tree) (not children))
      (flatten-task-tree-broad nil (list task) (list task))
      (let ((childs (mapcar #'cdr (reduce #'append (mapcar #'cpl:task-tree-node-children children)))))
        (if childs
            (flatten-task-tree-broad nil (append tree childs) childs)
            tree))))

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

(defun no-replacements (task)
  (let ((tasks (flatten-task-tree-broad task)))
    (remove nil (mapcar #'cpl-impl::task-tree-node-code-replacements tasks))))

(defun location-desig-dist (desig-1 desig-2)
  (cl-tf:v-dist (cl-tf:origin (reference desig-1))
                (cl-tf:origin (reference desig-2))))

(defun location-desig-close (desig-1 desig-2 &optional (threshold 0.9))
  (> threshold (location-desig-dist desig-1 desig-2)))

(defun search-tray ()
  (let ((tray (perform (an action
                           (type searching)
                           (object (an object (type :tray-box)))
                           (location (a location
                                        (on (desig:an object
                                                      (type counter-top)
                                                      (urdf-name kitchen_island_surface)
                                                      (owl-name "kitchen_island_counter_top")
                                                      (part-of kitchen)))
                                        (side back)))))))
    (when tray
      (cram-tf:pose-stamped->transform-stamped
       (cram-tf:ensure-pose-in-frame (obj-int:get-object-pose tray)
                                     cram-tf:*fixed-frame* :use-zero-time t)
       "tray_box_0"))))

(defun pose-on-tray (tray-transform obj-name)
  (let* ((frame-name (roslisp-utilities:rosify-underscores-lisp-name obj-name)))
    (destructuring-bind
        ((x y z) (ax ay az aw))
        (alexandria:assoc-value *tray-pose-transforms* obj-name)
      (cram-tf:multiply-transform-stampeds cram-tf:*fixed-frame*
                                           frame-name
                                           tray-transform
                                           (cl-tf:make-transform-stamped
                                            "tray_box_0" frame-name 0.0
                                            (cl-tf:make-3d-vector x y z)
                                            (cl-tf:make-quaternion ax ay az aw))
                                           :result-as-pose-or-transform :pose))))

(defun tray-transporting-action ()
  (let ((?pose (cl-tf:pose->pose-stamped "map" 0 
                                         (cram-tf:list->pose '((1.4 -0.2 1) (0 0 -0.7 0.7))))))
    (setf cram-mobile-pick-place-plans::*park-arms* nil)
    (perform (an action
                 (type transporting)
                 (object (an object 
                             (type tray-box)))
                 (location (a location
                              (on (desig:an object
                                            (type counter-top)
                                            (urdf-name kitchen-island-surface)
                                            (owl-name "kitchen_island_counter_top")
                                            (part-of kitchen)))))
                 (target (a location
                            (pose ?pose)))
                 (arm (left right))))
    (setf cram-mobile-pick-place-plans::*park-arms* t)))
