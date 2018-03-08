;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun make-side-costmap-generator (obj axis sign)
  "Returns a lambda function which for each (x y) gives 1.0
if it is on the sign side of the axis. "
  (when obj
    (let* ((bb-center (cl-transforms:origin (sem-map-utils:pose obj)))
           (bb-x (cl-transforms:x bb-center))
           (bb-y (cl-transforms:y bb-center)))
      (lambda (x y)
        (if (case axis
              (:x (funcall sign x bb-x))
              (:y (funcall sign y bb-y)))
            1.0
            0.0)))))

(defun make-restricted-area-cost-function ()
  (lambda (x y)
    (if (> x 1.0) 0.0
        (if (and (> x 0.0) (> y -1.0) (< y 1.0)) 1.0
            (if (and (< x 0.0) (> x -1.0) (> y -1.0) (< y 2.5)) 1.0
                0.0)))))

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'side-generator))) 5)
(defmethod location-costmap:costmap-generator-name->score ((name (eql 'restricted-area))) 5)

(def-fact-group demo-costmap (location-costmap:desig-costmap)
  (<- (location-costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:side :left))
    (desig:desig-prop ?designator (:on ?_))
    (desig:desig-prop ?designator (:name ?supp-obj-name))
    (lisp-fun sem-map-desig:designator->semantic-map-objects
              ?designator ?supp-objects)
    (member ?supp-object ?supp-objects)
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     side-generator
     (make-side-costmap-generator ?supp-object :y >)
     ?costmap))

  (<- (location-costmap:desig-costmap ?designator ?costmap)
    (or (cram-robot-interfaces:visibility-designator ?designator)
        (cram-robot-interfaces:reachability-designator ?designator))
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     restricted-area
     (make-restricted-area-cost-function)
     ?costmap)))

