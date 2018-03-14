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

(defvar *perform-counter* (make-hash-table :test 'equal))
(defvar *eval-list* nil)
(defvar *distributed-actions* (make-hash-table :test 'equal))
(defvar *previous-distributed-actions* (make-hash-table :test 'equal))
(defvar *deviation-table* (make-hash-table :test 'equal))

(defmethod exe:generic-perform :around ((designator action-designator))
  "Wraps the generic perform call to count execution times."
  (let ((action-type (desig-prop-value designator :type)))
    (unless (gethash action-type *perform-counter*)
      (setf (gethash action-type *perform-counter*) 0))
    (incf (gethash action-type *perform-counter*)))
    (call-next-method))

(defun with-evaluation (times function &rest args)
  (setf *eval-list* nil)
  (dotimes (c times)
    (roslisp-utilities:startup-ros)
    (setf *perform-counter* (make-hash-table :test 'equal))
    (apply function (car args) (cdr args))
    (push *perform-counter* *eval-list*))
  (eval-to-distribution))

(defun eval-to-distribution ()
  (setf *distributed-actions* (make-hash-table :test 'equal))
  (loop for key being the hash-keys in *perform-counter* do
    (setf (gethash key *distributed-actions*) '())
    (mapcar (lambda (table)
              (push (gethash key table) (gethash key *distributed-actions*)))
            *eval-list*)))

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


(defun t-value (devs)
  (let* ((sum-squared-deviations
           (- (reduce '+ (mapcar (alexandria:rcurry 'expt 2) devs))
              (/ (expt (reduce '+ devs) 2) (length devs))))
         (variance (/ sum-squared-deviations (1- (length devs))))
         (variance-distributed (sqrt (/ variance (length devs)))))
    (/ (/ (reduce '+ devs) (length devs)) variance-distributed)))
