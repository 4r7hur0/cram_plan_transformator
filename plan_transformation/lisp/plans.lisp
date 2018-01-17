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

(defun spawn-two-bottles (&optional (num-bottles 2))
  (unless (assoc :bottle btr::*mesh-files*)
    (add-objects-to-mesh-list))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         ,(when (<= 0 (decf num-bottles))
            `(assert (btr:object ?world :mesh bottle-1 ((-2 -0.9 0.860) (0 0 0 1))
                                :mass 0.2 :color (1 0 0) :mesh :bottle)))
         ;; (assert (btr:object ?world :mesh bottle-2 ((-0.8 2 0.9) (0 0 0 1))
         ;;                     :mass 0.2 :color (0 1 0) :mesh :bottle))
         ,(when (<= 0 (decf num-bottles))
            `(assert (btr:object ?world :mesh bottle-4 ((-1.9 -1 0.860) (0 0 0 1))
                                :mass 0.2 :color (1 0 0) :mesh :bottle)))
         ,(when (<= 0 (decf num-bottles))
            `(assert (btr:object ?world :mesh bottle-3 ((-1.8 -1.1 0.860) (0 0 0 1))
                                :mass 0.2 :color (1 0 0) :mesh :bottle)))
         
         (btr:simulate ?world 100))))

(def-cram-function navigate-to (?navigation-goal)
  (warn "Navigation executed")
  (exe:perform (desig:a motion
                        (type going)
                        (target (desig:a location (pose ?navigation-goal))))))

(defun look-at (?point-of-interest)
  (exe:perform (desig:a motion
                        (type looking)
                        (target (desig:a location (pose ?point-of-interest))))))

(defun get-perceived-bottle-desig ()
  (let* ((?bottle-desig (desig:an object (type bottle)
                                  ;; (color (1 0 0))
                                  ))
         (?perceived-bottle-desig (desig:a motion
                                            (type detecting)
                                            (object ?bottle-desig)
                                            )))
    (exe:perform ?perceived-bottle-desig)))

(def-cram-function pick-up (?object-designator &optional (?arm :right))
  (unless (member ?arm (available-arms))
    (cpl:fail 'cpl-impl:plan-failure))
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

(defun navigate1 ()
  (warn "In navigate 1: ~a" cpl:*current-path*)
  (let ((?navigation-goal *pose-counter*))
    (navigate-to ?navigation-goal))
  ;; (replace-task-code '(navigate3) #'navigate3 ;; '((NAVIGATE1) (CRAM-LANGUAGE-IMPLEMENTATION:TOP-LEVEL TT))
  ;;                    cpl:*current-path*)
  )

(defun navigate2 ()
  (warn "In navigate 2: ~a" cpl:*current-path*)
  (let ((?pose *pose-meal-table*))
    (navigate-to ?pose))
  ;; (replace-task-code '(printme) #'(lambda () (printme "Oranges")) cpl:*current-path*)
  )

(def-cram-function navigate3 ()
  (warn "In navigate 3: ~a" cpl:*current-path*)
  (let ((?pose *pose-meal-table*))
    (navigate-to ?pose))
  ;; (replace-task-code '(navigate1) #'navigate1 cpl:*current-path*)
  )

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

(def-cram-function just-print ()
  (warn "about something"))

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

(def-cram-function pick-up-one-bottle (&optional (arm (first (available-arms))))
  (when (null arm)
    (cpl:fail 'cpl-impl:plan-failure))
  (navigate1)
  (let ((?ptu-goal *ptu-goal*))
    (look-at ?ptu-goal))
  (let ((?perceived-bottle-1 (get-perceived-bottle-desig))
        (?arm arm))
    (pick-up ?perceived-bottle-1 ?arm)
    (pr2-pp-plans::park-arms :arm ?arm)))

(def-cram-function bring-one-by-one () 
  (cpl:with-failure-handling
      ((cram-common-failures:perception-object-not-found (f)
         (navigate1)
         (let ((?ptu-goal *ptu-goal*))
           (look-at ?ptu-goal))
         )
       (cpl-impl:plan-failure (e)
         (navigate2)))
    (progn
      (loop for i to 2
            do 
      (fetch-bottle-from-counter (first (available-arms)))))))

(def-cram-function bring-at-once () 
  (cpl:with-failure-handling
      ((cram-common-failures:perception-object-not-found (f)
         (declare (ignore f))
         (navigate1)
         (let ((?ptu-goal *ptu-goal*))
           (look-at ?ptu-goal)))
       (cpl-impl:plan-failure (e)
         (declare (ignore e))
         (warn "Failure handling is ~a" cpl:*current-path*)
         (replace-task-code '(bring-one-by-one) #'bring-one-by-one
                            (cdr cpl:*current-path*))
         ;; (reset 'nsdsil)
         ;; (spawn-two-bottles 3)
         ;; (prepare-pr2)
         (return)
         
         ))
    (progn
      (loop for i to 2
             do (pick-up-one-bottle (first (available-arms))))
      (warn "The path in execution is ~a" cpl:*current-path*))))
