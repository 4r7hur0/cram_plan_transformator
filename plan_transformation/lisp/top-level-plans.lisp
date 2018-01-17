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

(defun tl-replacement-example ()
  (replace-task-code (format t "wololo") (lambda () (format t "wololo"))
                     '((cpl:top-level tt2))
                     (cpl:get-top-level-task-tree 'tt2)))


(def-top-level-cram-function tt2 ()
  (spawn-two-bottles 3)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    ;;(bring-one-by-one)
    ;; (prepare-pr2)
    (bring-at-once)
    ))

(def-top-level-cram-function tt ()
  (spawn-two-bottles)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    ;; Go to counter top and perceive bottle
    (prepare-pr2)
    ;; (fetch-bottle-from-counter)
    (fetch-both-bottles)))  

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
