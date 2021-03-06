;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(defparameter *location-distance-threshold* 0.5)

(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))
(defparameter *island-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *object-spawning-poses*
  '((:breakfast-cereal . ((1.3 1.0 0.95) (0 0 0 1)))
    (:cup . ((1.35 0.7 0.9) (0 0 0 1)))
    (:bowl . ((1.4 0.8 0.97) (0 0 0 1)))
    (:spoon . ((1.4 0.35 0.85) (0 0 0 1)))
    (:milk . ((1.3 0.50 0.95) (0 0 0 1)))
    (:tray . ((1.45 0.0 0.94) (0 0 -0.7071 0.7071)))
    (:tray-base . ((1.45 0.0 0.85) (0 0 0 1)))))

(defparameter *object-spawning-in-container-poses*
  '((:breakfast-cereal . ((1.45 -0.95 0.96) (0 0 0 1)))
    (:cup . ((1.45 -1.1 0.9) (0 0 0 1)))
    (:bowl . ((1.4 0.85 0.97) (0 0 0 1)))
    (:spoon . ((1.4 0.4 0.85) (0 0 0 1)))
    (:milk . ((1.45 -1.2 0.95) (0 0 0 1)))
    (:tray . ((1.45 0.0 0.94) (0 0 -0.7071 0.7071)))
    (:tray-base . ((1.45 0.0 0.85) (0 0 0 1)))))

(defparameter *object-grasping-arms*
  '((:breakfast-cereal . :left)
    (:cup . :left)
    (:bowl . :right)
    (:spoon . :right)
    (:milk . :right)
    (:tray . :right)))

(defparameter *object-placing-poses*
  '((:breakfast-cereal .  ((-0.78 0.9 0.95) (0 0 1 0)))
    (:cup . ((-0.79 1.35 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.76 1.19 0.93) (0 0 0.7071 0.7071)))
    (:spoon . ((-0.78 1.5 0.86) (0 0 0 1)))
    (:milk . ((-0.7 1.7 0.96) (0 0 0.7071 0.7071)
              ))
    (:tray . ((-0.78 1.85 0.9) (0 0 1 0)))))

(defparameter *tray-pose-transforms*
  '((:breakfast-cereal .  ((-0.12 0.04 0.14)(0 0 0 1))) ;;left
    (:cup . ((-0.15 0.17 0.1)(0 0 0 1))) ;;left
    (:bowl . ((-0.12 0.04 0.14)(0 0 0 1)))
    (:spoon . ((-0.20 0.04 0.12)(0 0 0 1)))
    (:milk . ((-0.12 -0.06 0.16) (0 0 1 0))) ;;right
    ;; (:tray . ((-0.75 1.85 0.85) (0 0 1 0)))
    ))


(defun spawn-objects-on-sink-counter (&optional (spawning-poses *object-spawning-poses*))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  (let ((object-types '(:tray-base
                        :tray
                        :breakfast-cereal
                        :spoon
                        :milk
                        :cup
                        ;; :bowl
                        )))
    ;; spawn objects at default poses
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type
                              :pose (cdr (assoc object-type spawning-poses))))
                           object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      objects)))

(defun spawn-objects-on-sink-counter-randomly ()
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (let ((object-types '(:cereal :cup :bowl :spoon :milk)))
    ;; spawn at default location
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type))
                           object-types)))
      ;; move on top of counter tops
      (mapcar (lambda (btr-object)
                (let* ((aabb-z (cl-transforms:z
                                (cl-bullet:bounding-box-dimensions (btr:aabb btr-object))))
                       (new-pose (cram-tf:rotate-pose
                                  (cram-tf:translate-pose
                                   (desig:reference
                                    (desig:a location
                                             (on "CounterTop")
                                             (name "iai_kitchen_sink_area_counter_top")
                                             (side left)
                                             (centered-with-padding 0.1)))
                                   :z-offset (/ aabb-z 2.0))
                                  :z (/ pi (random 10.0)))))
                  (btr-utils:move-object (btr:name btr-object) new-pose)))
              objects)))
  ;; stabilize world
  (btr:simulate btr:*current-bullet-world* 100))

(defun spawn-objects-into-fridge ()
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  (flet ((move-into-fridge (type)
           (let ((name (intern (format nil "~a-1" type) :keyword)))
             (btr-utils:spawn-object name type :pose (cdr (assoc type *object-spawning-poses*)))
             (setf (btr::mass (car (btr:rigid-bodies (btr:object btr:*current-bullet-world* name)))) 0.0)
             (btr-utils:move-object name (cdr (assoc type *object-spawning-in-container-poses*))))))
    (let* ((object-types '(;; :tray-base
                           ;; :tray
                           :breakfast-cereal
                           ;; :spoon
                           :milk
                           :cup
                           ;; :bowl
                        )))
      (mapcar #'move-into-fridge object-types))))

(defun spawn-objects-mixed ()
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  (flet ((move-into-fridge (type stay-on-sink)
           (let ((name (intern (format nil "~a-1" type) :keyword)))
             (btr-utils:spawn-object name type :pose (cdr (assoc type *object-spawning-poses*)))
             (unless (member type stay-on-sink)
               (setf (btr::mass (car (btr:rigid-bodies (btr:object btr:*current-bullet-world* name)))) 0.0)
               (btr-utils:move-object name (cdr (assoc type *object-spawning-in-container-poses*)))))))
    
    (let* ((object-types '(:breakfast-cereal :spoon :milk :cup
                           :tray-base :tray
                           ;; :bowl
                           ))
           (stay-on-sink '(:breakfast-cereal :spoon :tray-base :tray)))
           (mapcar (alexandria:rcurry #'move-into-fridge stay-on-sink) object-types))))

;; (defun go-to-sink-or-island (&optional (sink-or-island :sink))
;;   (let ((?navigation-goal (ecase sink-or-island
;;                             (:sink *sink-nav-goal*)
;;                             (:island *island-nav-goal*)))
;;         (?ptu-goal *look-goal*))
;;     (cpl:par
;;       (pp-plans::park-arms)
;;       (exe:perform (desig:a motion
;;                             (type going)
;;                             (target (desig:a location (pose ?navigation-goal))))))
;;     (exe:perform (desig:a motion
;;                           (type looking)
;;                           (target (desig:a location (pose ?ptu-goal)))))))

;; (defun pick-object (&optional (?object-type :breakfast-cereal) (?arm :right))
;;   (pp-plans:park-arms)
;;   (go-to-sink-or-island :sink)
;;   (let* ((?object-desig
;;            (desig:an object (type ?object-type)))
;;          (?perceived-object-desig
;;            (exe:perform (desig:an action
;;                                   (type detecting)
;;                                   (object ?object-desig)))))
;;     (cpl:par
;;       (exe:perform (desig:an action
;;                              (type looking)
;;                              (object ?perceived-object-desig)))
;;       (exe:perform (desig:an action
;;                              (type picking-up)
;;                              (arm ?arm)
;;                              (object ?perceived-object-desig))))))

;; (defun place-object (?target-pose &optional (?arm :right))
;;   (pp-plans:park-arms)
;;   (go-to-sink-or-island :island)
;;   (cpl:par
;;     (exe:perform (desig:a motion
;;                           (type looking)
;;                           (target (desig:a location
;;                                            (pose ?target-pose)))))
;;     (exe:perform (desig:an action
;;                            (type placing)
;;                            (arm ?arm)
;;                            (target (desig:a location
;;                                             (pose ?target-pose)))))))

;; (defun demo-hard-coded ()
;;   (spawn-objects-on-sink-counter)

;;   (pr2-proj:with-simulated-robot

;;     (dolist (object-type '(:breakfast-cereal :cup :bowl :spoon :milk))

;;       (let ((placing-target
;;               (cl-transforms-stamped:pose->pose-stamped
;;                "map" 0.0
;;                (cram-bullet-reasoning:ensure-pose
;;                 (cdr (assoc object-type *object-placing-poses*)))))
;;             (arm-to-use
;;               (cdr (assoc object-type *object-grasping-arms*))))

;;         (pick-object object-type arm-to-use)
;;         (place-object placing-target arm-to-use)))))

