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

(defun init-environment ()
  (def-fact-group costmap-metadata ()
    (<- (location-costmap:costmap-size 12 12))
    (<- (location-costmap:costmap-origin -6 -6))
    (<- (location-costmap:costmap-resolution 0.05))

    (<- (location-costmap:costmap-padding 0.2))
    (<- (location-costmap:costmap-manipulation-padding 0.2))
    (<- (location-costmap:costmap-in-reach-distance 0.6))
    (<- (location-costmap:costmap-reach-minimal-distance 0.2)))

  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "kitchen_description")

  (sem-map:get-semantic-map)

  (cram-occasions-events:clear-belief)

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t))

(roslisp-utilities:register-ros-init-function init-environment)

(defun reset (&optional (task-tree-name 'tt2))
  (btr-utils:kill-object 'CRAM-PR2-DESCRIPTION:PR2)
  (btr-utils:kill-object 'bottle-1)
  (btr-utils:kill-object 'bottle-3)
  (btr-utils:kill-object 'bottle-4)
  (let ((robot-urdf
                   (cl-urdf:parse-urdf
                    (roslisp:get-param "robot_description"))))
             (prolog:prolog
              `(and (btr:bullet-world ?world)
                    (cram-robot-interfaces:robot ?robot)
                    (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
                    (cram-robot-interfaces:robot-arms-parking-joint-states ?robot ?joint-states)
                    (assert (btr:joint-state ?world ?robot ?joint-states))
                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.22d0))))
                    )))
  ;; (cram-occasions-events:clear-belief)
  (when task-tree-name
    (cpl-impl::remove-top-level-task-tree task-tree-name)))

;; (defmethod location-costmap:on-visualize-costmap opengl ((map location-costmap:location-costmap))
;;   (btr:add-costmap-function-object map))

;; (defmethod location-costmap:on-visualize-costmap-sample opengl ((point cl-transforms:3d-vector))
;;   (btr:add-costmap-sample-object point))
