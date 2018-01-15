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

(defparameter *pose-bottle-1*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 -0.9d0 0.86d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *pose-bottle-2*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -0.8 2 0.9)
   (cl-transforms:make-identity-rotation)))

(defparameter *pose-bottle-3*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2 -1.1d0 0.86d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *pose-meal-table*
  (cl-tf:make-pose-stamped
   "map" 0.0
   (cl-tf:make-3d-vector -0.15 2.0 0)
   (cl-tf:make-quaternion 0.0d0 0.0d0 -1.0d0 0.0d0)))

(defparameter *pose-counter*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -1.8547d0 -0.381d0 0.0d0)
   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))

(defparameter *ptu-goal*
  (cl-transforms-stamped:make-pose-stamped
             "base_footprint"
             0.0
             (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
             (cl-transforms:make-identity-rotation)))

(def-cram-function spawn-two-bottles ()
  (unless (assoc :bottle btr::*mesh-files*)
    (add-objects-to-mesh-list))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (assert (btr:object ?world :mesh bottle-1 ((-2 -0.9 0.860) (0 0 0 1))
                             :mass 0.2 :color (1 0 0) :mesh :bottle))
         ;; (assert (btr:object ?world :mesh bottle-2 ((-0.8 2 0.9) (0 0 0 1))
         ;;                     :mass 0.2 :color (0 1 0) :mesh :bottle))
         (assert (btr:object ?world :mesh bottle-3 ((-1.8 -0.9 0.860) (0 0 0 1))
                             :mass 0.2 :color (1 0 0) :mesh :bottle))
         (btr:simulate ?world 100))))

(def-cram-function navigate-to (?navigation-goal)
  (warn "Navigation executed")
  (exe:perform (desig:a motion
                        (type going)
                        (target (desig:a location (pose ?navigation-goal))))))

(def-cram-function look-at (?point-of-interest)
  (exe:perform (desig:a motion
                        (type looking)
                        (target (desig:a location (pose ?point-of-interest))))))

(def-cram-function get-perceived-bottle-desig ()
  (let* ((?bottle-desig (desig:an object (type bottle)
                                  (color (1 0 0))))
         (?perceived-bottle-desig (desig:a motion
                                            (type detecting)
                                            (object ?bottle-desig)
                                            )))
    (exe:perform ?perceived-bottle-desig)))

(def-cram-function pick-up (?object-designator &optional (?arm :right))
  (exe:perform (desig:an action
                         (type picking-up)
                         (arm ?arm)
                         (object ?object-designator))))

(def-cram-function place-down (?pose ?object ?arm)
  (exe:perform (desig:an action
                         (type placing)
                         (arm ?arm)
                         (object ?object)
                         (target (desig:a location (pose ?pose))))))

(def-cram-function navigate1 ()
  (warn "In navigate 1: ~a" cpl:*current-path*)
  (let ((?navigation-goal *pose-counter*))
    (navigate-to ?navigation-goal))
  ;; (replace-task-code '(navigate3) #'navigate3 ;; '((NAVIGATE1) (CRAM-LANGUAGE-IMPLEMENTATION:TOP-LEVEL TT))
  ;;                    cpl:*current-path*)
  )

(def-cram-function navigate2 ()
  (warn "In navigate 2: ~a" cpl:*current-path*)
  (let ((?pose *pose-meal-table*))
    (navigate-to ?pose))
  ;; (replace-task-code '(printme) #'(lambda () (printme "Oranges")) cpl:*current-path*)
  )

(def-cram-function navigate3 ()
  (warn "In navigate 3: ~a" cpl:*current-path*)
  (let ((?pose *pose-meal-table*))
    (navigate-to ?pose))
  (replace-task-code '(navigate1) #'navigate1 cpl:*current-path*))

(def-cram-function printme (arg)
  (format T "This is the last call! And ~a" arg)
  (format T "This task is ~a" (cpl:task cpl:*current-path*))
  (format T "And this ~a" cpl:*current-task*))


(def-cram-function prepare-pr2 ()
  (cpl:par
    ;; Move torso up
    (exe:perform
     (desig:a motion (type moving-torso) (joint-angle 0.3)))
    ;; Park arms
    (pr2-pp-plans::park-arms)))

(def-cram-function fetch-bottle-from-counter (&optional (arm :right))
  (warn "~a" cpl:*current-path*)
  (navigate1)
  (let ((?ptu-goal *ptu-goal*))
    (look-at ?ptu-goal))
  (let ((?perceived-bottle-1 (get-perceived-bottle-desig))
        (?arm arm))
    (pick-up ?perceived-bottle-1 ?arm)
    (pr2-pp-plans::park-arms :arm arm)
    (navigate2)
    (let ((?drop-pose *pose-bottle-2*))
      (place-down ?drop-pose ?perceived-bottle-1 arm)
      (pr2-pp-plans::park-arms :arm arm)))
  ;; (replace-task-code '(fetch-bottle-from-counter :left) #'(lambda () (fetch-bottle-from-counter :left))
  ;;                    cpl:*current-path*)
  )

(def-cram-function fetch-both-bottles ()
  (navigate1)
  (let ((?ptu-goal *ptu-goal*))
    (look-at ?ptu-goal))
  (let ((?perceived-bottle-r (get-perceived-bottle-desig)))
    (pick-up ?perceived-bottle-r :right)
    (pr2-pp-plans::park-arms :arm :right)
    (let ((?perceived-bottle-l (get-perceived-bottle-desig))) 
          (pick-up ?perceived-bottle-l :left)
          (pr2-pp-plans::park-arms :arm :left)
          (navigate2)
          (let ((?drop-pose *pose-bottle-2*))
            (place-down ?drop-pose ?perceived-bottle-r :right)
            (pr2-pp-plans::park-arms :arm :right)
            (place-down ?drop-pose ?perceived-bottle-l :left)
            (pr2-pp-plans::park-arms :arm :left))))
  (replace-task-code '(fetch-bottle-from-counter :left) #'(lambda () (fetch-bottle-from-counter :left))
                     cpl:*current-path*))

(def-top-level-cram-function tt ()
  (spawn-two-bottles)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    ;; Go to counter top and perceive bottle
    (prepare-pr2)
    (fetch-bottle-from-counter)
    (fetch-bottle-from-counter)
    
;;    (fetch-both-bottles)
    ;; Pick up bottle-2 with left arm
    ;; (let ((?perceived-bottle-2 (get-perceived-bottle-desig)))
    ;;   (pick-up ?perceived-bottle-2 :left)
    ;;   ;; Move left arm out of sight
    ;;   (pr2-pp-plans::park-arms :arm :left)
      ;; Place bottle-1 on second table
      ;; (let ((?drop-pose *pose-bottle-2*))
      ;;   (place-down ?drop-pose ?perceived-bottle-1 :right)
      ;; ;; Move right arm out of sight
      ;;   (pr2-pp-plans::park-arms :arm :right)
      ;; ;; Move to the counter table 
      ;;   (navigate1)
      ;;   (let ((?perceived-bottle-3 (get-perceived-bottle-desig)))
      ;;     (pick-up ?perceived-bottle-3 :left)
      ;;     (pr2-pp-plans::park-arms :arm :left)
      ;;     (let ((?pose *pose-meal-table*))
      ;;       (navigate-to ?pose))
      ;;     (place-down ?drop-pose ?perceived-bottle-1 :left))
      ;; ;; Place bottle-2 on the counter
      ;; ;; (let ((?drop-pose *pose-bottle-1*))
      ;; ;;   (place-down ?drop-pose ?perceived-bottle-2 :left))
      ;; (pr2-pp-plans::park-arms))
    ))

    

(def-top-level-cram-function test-switch-two-bottles ()
  (spawn-two-bottles)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    
      ;; Go to counter top and perceive bottle
      (let ((?navigation-goal *pose-counter*)
            (?ptu-goal 
              (cl-transforms-stamped:make-pose-stamped
               "base_footprint"
               0.0
               (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
               (cl-transforms:make-identity-rotation))))
        (cpl:par
          ;; Move torso up
          (exe:perform
           (desig:a motion (type moving-torso) (joint-angle 0.3)))
          (pr2-pp-plans::park-arms)
          (navigate-to ?navigation-goal))
        (look-at ?ptu-goal))
      ;; Pick up bottle-1 with right arm.
      (let ((?perceived-bottle-1 (get-perceived-bottle-desig)))
        (pick-up ?perceived-bottle-1 :right)
        (pr2-pp-plans::park-arms :arm :right)
        ;; Move to the meal table
        (let ((?pose *pose-meal-table*))
          (navigate-to ?pose))
        ;; Pick up bottle-2 with left arm
        (let ((?perceived-bottle-2 (get-perceived-bottle-desig)))
          (pick-up ?perceived-bottle-2 :left)
          ;; Move left arm out of sight
          (pr2-pp-plans::park-arms :arm :left)
          ;; Place bottle-1 on second table
          (let ((?drop-pose *pose-bottle-2*))
            (place-down ?drop-pose ?perceived-bottle-1 :right))
          ;; Move right arm out of sight
          (pr2-pp-plans::park-arms :arm :right)
          ;; Move to the counter table 
          (let ((?navigation-goal *pose-counter*))
            (navigate-to ?navigation-goal))
          ;; Place bottle-2 on the counter
          (let ((?drop-pose *pose-bottle-1*))
            (place-down ?drop-pose ?perceived-bottle-2 :left))
          (pr2-pp-plans::park-arms)))))
