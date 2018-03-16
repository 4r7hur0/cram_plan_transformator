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

(defvar *all-data-store* '())
(defvar *perform-counter* (make-hash-table :test 'equal))
(defvar *action-list* '())
(defvar *deviation-table* (make-hash-table :test 'equal))

(defvar *distributed-actions* (make-hash-table :test 'equal))
(defvar *motion-cost* (make-hash-table :test 'equal))

(defparameter *home-pose-right*
  (cl-tf:make-pose-stamped
   "base_footprint" 0.0
   (cl-tf:make-3d-vector 0.25 -0.5 1.3)
   (cl-tf:make-quaternion 0.0d0 0.0d0 0.15643446735208227d0 0.9876883402289764d0)))

(defparameter *home-pose-left*
  (cl-tf:make-pose-stamped
   "base_footprint" 0.0
   (cl-tf:make-3d-vector 0.25 0.5 1.3)
   (cl-tf:make-quaternion 0.0d0 0.0d0 -0.15643446735208227d0 0.9876883402289764d0)))


(defparameter *pre-action* nil)
(defparameter *post-action* nil)

(defmethod exe:generic-perform :around ((designator action-designator))
  "Wraps the generic perform call to count execution times."
  (let ((action-type (desig-prop-value designator :type)))
    (unless (gethash action-type *perform-counter*)
      (setf (gethash action-type *perform-counter*) 0))
    (incf (gethash action-type *perform-counter*)))
  (call-next-method))

(defun with-evaluation (times function &rest args)
  (reset-before-whole-testrun)
  (dotimes (c times)
    (roslisp-utilities:startup-ros)
    (setf *perform-counter* (make-hash-table :test 'equal))
    (apply function (car args) (cdr args))
    (store-current-run-data))
  (store-after-whole-testrun))

(defun reset-before-whole-testrun ()
  (setf *eval-list* nil)
  (setf *motion-cost* (make-hash-table :test 'equal))
  (setf *distributed-actions* (make-hash-table :test 'equal)))

(defun store-after-whole-testrun ()
  (push-data-to-action-distribution-table)
  (push (list *distributed-actions* *motion-cost*) *all-data-store*))

(defun store-current-run-data ()
  (push *perform-counter* *action-list*)
  (push (total-distance-navigated) (gethash :navigated *motion-cost*))
  (push (total-distance-gripper-moved) (gethash :gripper-movement *motion-cost*))
  (push (total-time-container-manipulated) (gethash :container *motion-cost*)))

(defun push-data-to-action-distribution-table ()
  (setf *distributed-actions* (make-hash-table :test 'equal))
  (loop for key being the hash-keys in *perform-counter* do
    (setf (gethash key *distributed-actions*) '())
    (mapcar (lambda (table)
              (push (gethash key table) (gethash key *distributed-actions*)))
            *action-list*)))

(defun calculate-deviations (table1 table2)
  (loop for key being the hash-keys in table1 do
    (setf (gethash key *deviation-table*)
          (loop for i to (1- (length (gethash key table1)))
                collect (let ((val1 (nth i (gethash key table1)))
                              (val2 (nth i (gethash key table2))))
                          (if (and val1 val2)
                              (- val1 val2)
                              0))))))
            
(defun calculate-overall-deviation (&optional (dev-table *deviation-table*))
  (loop for key being the hash-keys in dev-table
           collect (float (alexandria:mean (gethash key dev-table)))))

(defun total-distance-navigated ()
  (let* ((lazy-motions
           (prolog '(and
                     (top-level-name ?top-level-name)
                     (top-level-path ?path)
                     (task-specific-motion ?top-level-name ?path :going ?_ ?desig))))
         (motions (sort
                   (mapcar (alexandria:rcurry #'alexandria:assoc-value '?desig)
                           (cut:force-ll lazy-motions))
                   #'< :key #'timestamp))
         (poses (mapcar (alexandria:rcurry #'get-pose-from-motion :target)
                        motions)))
    (reduce #'+ (loop for i to (- (length poses) 2)
                      collect (estimate-distance-between-pose-stamped
                               (nth i poses) (nth (1+ i) poses))))))

(defun get-pose-from-motion (motion accessor)
  (let ((target (desig-prop-value motion accessor)))
    (if (desig-prop-value target :pose)
        (desig-prop-value target :pose)
        (desig-prop-value (desig-prop-value target :location) :pose))))

(defun get-gripper-pose (motion side)
  (let ((location (car (remove nil (list (desig-prop-value motion :right-target)
                                        (desig-prop-value motion :left-target))))))
    (if location
        (desig-prop-value location :pose)
        (case side
          (:right *home-pose-right*)
          (:left *home-pose-left*)))))
    

(defun total-distance-gripper-moved ()
  (let* ((lazy-gripper-motions
           (prolog '(and
                     (top-level-name ?top-level-name)
                     (top-level-path ?path)
                     (task-specific-motion ?top-level-name ?path :moving-tcp ?_ ?desig))))
         (all-gripper-motions
           (sort (mapcar (alexandria:rcurry #'alexandria:assoc-value '?desig)
                         (cut:force-ll lazy-gripper-motions))
                 #'< :key #'timestamp))
         (left-gripper-poses
           (mapcar (alexandria:rcurry #'get-gripper-pose :left)
                   (remove-if (alexandria:rcurry #'desig-prop-value :right-target)
                                  all-gripper-motions)))
         (right-gripper-poses
           (mapcar (alexandria:rcurry #'get-gripper-pose :right)
                   (remove-if (alexandria:rcurry #'desig-prop-value :left-target)
                                  all-gripper-motions))))
   
    (+ (reduce #'+ (loop for i to (- (length left-gripper-poses) 2)
                      collect (estimate-distance-between-pose-stamped
                               (nth i left-gripper-poses) (nth (1+ i) left-gripper-poses))))
       (reduce #'+ (loop for i to (- (length right-gripper-poses) 2)
                         collect (estimate-distance-between-pose-stamped
                                  (nth i right-gripper-poses) (nth (1+ i) right-gripper-poses)))))))

(defun total-time-container-manipulated ()
  (let* ((accessing-actions
           (cut:force-ll
            (prolog '(and
                      (top-level-name ?top-level-name)
                      (top-level-path ?path)
                      (task-specific-action ?top-level-name ?path :closing-container ?closing ?_)))))
         (closing-actions
           (cut:force-ll
            (prolog '(and
                      (top-level-name ?top-level-name)
                      (top-level-path ?path)
                      (task-specific-action ?top-level-name ?path :closing-container ?closing ?_))))))

    (* (length (append accessing-actions closing-actions)) 5)))
  
(defun estimate-distance-between-pose-stamped (pose-stamped-1 pose-stamped-2
                                               &optional
                                                 (translational-weight 1.0)
                                                 (rotational-weight 1.0))
  "From Gaya"
  (assert pose-stamped-1)
  (assert pose-stamped-2)
  (assert (string-equal (cl-transforms-stamped:frame-id pose-stamped-1)
                        (cl-transforms-stamped:frame-id pose-stamped-2)))
  (+ (* translational-weight
        (cl-transforms:v-dist
         (cl-transforms:origin pose-stamped-1)
         (cl-transforms:origin pose-stamped-2)))
     (* rotational-weight
        (abs
         (cl-transforms:normalize-angle
          (cl-transforms:angle-between-quaternions
           (cl-transforms:orientation pose-stamped-1)
           (cl-transforms:orientation pose-stamped-2)))))))

(defun t-value (devs)
  (let* ((sum-squared-deviations
           (- (reduce '+ (mapcar (alexandria:rcurry 'expt 2) devs))
              (/ (expt (reduce '+ devs) 2) (length devs))))
         (variance (/ sum-squared-deviations (1- (length devs))))
         (variance-distributed (sqrt (/ variance (length devs)))))
    (/ (/ (reduce '+ devs) (length devs)) variance-distributed)))
