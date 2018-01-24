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

(defun get-kitchen-urdf ()
  (slot-value
   (btr:object btr:*current-bullet-world* :kitchen)
   'cram-bullet-reasoning:urdf))

(defun move-kitchen-joint (&key (joint-name "iai_fridge_door_joint")
                             (joint-angle 0.2d0) (kitchen-name :kitchen))
  (btr:set-robot-state-from-joints
   `((,joint-name  ,joint-angle))
   (btr:object btr:*current-bullet-world* kitchen-name)))

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

(defun get-tltt (tltt-name)
  (cpl:get-top-level-task-tree tltt-name))

(defun available-arms ()
  (mapcar 'cdr
          (let ((link-list '(("l_wrist_roll_link" . :left) ("r_wrist_roll_link" . :right))))
            (set-difference link-list (mapcar (lambda (entry) (btr::attachment-link (caadr entry)))
                                              (btr:attached-objects (btr:get-robot-object)))
                            :test #'(lambda (a1 a2) (string= (car a1) a2))))))

(defun get-children (node)
       (if (null (cpl:task-tree-node-children node))
           (first (cpl:task-tree-node-path node))
           (append (list (first (cpl:task-tree-node-path node)))
                   (list (mapcar (lambda (child-par) (get-children (cdr child-par)))
                                 (cpl:task-tree-node-children node))))))

(defun find-task (task-name node)
  (let ((matches '()))
    (if (and node (equal task-name (first (cpl:task-tree-node-path node))))
             (setf matches (append matches (list node)))
             (setf matches (append matches (map 'list (lambda (child-par) (find-task task-name (cdr child-par)))
                                  (cpl:task-tree-node-children node)))))
    (alexandria:flatten matches)))

(defun get-robot-position (node)
  (first (cpl-impl:code-parameters (cpl:task-tree-node-code (cdr (find-if (lambda (entry) (equal (car entry) '(navigate-to)))
           (cpl:task-tree-node-children (cpl:task-tree-node-parent (cpl:task-tree-node-parent node)))))))))


(defun reset-fast ()
  (reset)
  (tt2)
  (reset nil)
  (apply-rules)
  (tt2))
