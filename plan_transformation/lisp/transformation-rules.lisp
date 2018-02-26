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

(defun apply-rules (&optional (tltt (cpl:get-top-level-task-tree 'tt2)))
  (apply-double-grab-dumb tltt))

(defun apply-double-grab-dumb (tltt)
  (let ((pick-list (find-task '(cram-mobile-pick-place-plans:pick-up) tltt)))
    (if (equal (get-robot-position (first pick-list)) (get-robot-position (second pick-list)))
        (replace-task-code '(BRING-AT-ONCE) #'bring-at-once
                           '((BRING-ONE-BY-ONE) (CRAM-LANGUAGE-IMPLEMENTATION:TOP-LEVEL TT2)) tltt)
        (warn "rule not applicable"))))

(defun get-location-from-task (task)
  (desig:description
   (desig:desig-prop-value 
    (cut:var-value '?desig task) :location)))

(defun tasks-with-matching-location (&optional (top-level-name :top-level))
  (let* ((transporting-tasks
           (cut:force-ll
            (prolog:prolog `(task-transporting-action ,top-level-name ((demo-random))
                                                      ?task ?desig))))
        (match)
        (matching-pairs (list)))
    (loop while transporting-tasks
          do (when (setf match
                         (find-if (lambda (x) (equalp (get-location-from-task (car transporting-tasks))
                                                      (get-location-from-task x))) (cdr transporting-tasks)))
               (push (list (car transporting-tasks) match) matching-pairs))
             (setf transporting-tasks (remove match (cdr transporting-tasks))))
    matching-pairs))

(defun action-designator-under-path (path action-type &optional (top-level-name :top-level))
  (alexandria:assoc-value
   (car (cut:force-ll 
         (prolog:prolog `(and
                          (task-specific-action ,top-level-name ,path ,action-type ?task ?desig))))) '?desig))

(defun pose->btr-pose (pose)
  (declare (type cl-tf:pose pose))
  (let* ((origin (cl-tf:origin pose))
         (orientation (cl-tf:orientation pose))
         (x (cl-tf:x origin))
         (y (cl-tf:y origin))
         (z (cl-tf:z origin))
         (ow (cl-tf:w orientation))
         (ox (cl-tf:x orientation))
         (oy (cl-tf:y orientation))
         (oz (cl-tf:z orientation)))
    (list (list x y z) (list ox oy oz ow))))


(defun test-transporting-query ()
  (cut:force-ll (prolog:prolog
       `(and
         (task-similar-transporting-action :top-level ((demo-random)) ?other ?loc)))))

(defun both-hands-transporting-rule (&optional (top-level-name :top-level))
  (let* ((transporting-tasks
           (cut:force-ll
            (prolog:prolog
             `(task-specific-action ,top-level-name ((demo-random))
                                    :transporting ?task ?desig))))
        (match)
        (matching-pairs (list)))
    (loop while transporting-tasks
          do (when (setf match
                         (find-if (lambda (x) (equalp (get-location-from-task (car transporting-tasks))
                                                      (get-location-from-task x))) (cdr transporting-tasks)))
               (push (list (car transporting-tasks) match) matching-pairs))
             (setf transporting-tasks (remove match (cdr transporting-tasks))))
    (let* ((tasks (mapcar (alexandria:rcurry #'alexandria:assoc-value '?task) (car matching-pairs)))
           (paths (mapcar #'cpl:task-tree-node-path tasks))
           (first-fetching-desig (action-designator-under-path (cadr paths) :fetching top-level-name))
           (first-delivery-desig (action-designator-under-path (cadr paths) :delivering top-level-name)))
      (cpl-impl::replace-task-code '(TRANSFORM1)
                                   #'(lambda (&rest desig)
                                       (declare (ignore desig))
                                       (exe:perform first-fetching-desig))
                                   (cadr paths) (cpl-impl::get-top-level-task-tree top-level-name))
      (cpl-impl::replace-task-code '(TRANSFORM2)
                                   #'(lambda (&rest desig)
                                       (exe:perform (car desig))
                                       (exe:perform first-delivery-desig))
                                   (car paths) (cpl-impl::get-top-level-task-tree top-level-name)))))

(defun tray-transporting-action ()
  (let* ((?tray-obj (desig:an object
                             (type :tray)))
        (?fetching-location
          (desig:a location
                   (on "CounterTop")
                   (name "iai_kitchen_sink_area_counter_top")))
        (?placing-target-pose
          (cl-transforms-stamped:pose->pose-stamped
           "map" 0.0
           (cram-bullet-reasoning:ensure-pose
            '((-0.75 1.85 0.85) (0 0 1 0)))))
        (?delivering-location
          (desig:a location
                   (pose ?placing-target-pose)))
        (action (desig:an action
                          (type transporting)
                          (object ?tray-obj)
                          (arm :right)
                          (location ?fetching-location)
                          (target ?delivering-location)
                          (retract-arms nil))))
    action))
  
(defun stacking-rule (&optional (top-level-name :top-level))
  (let* ((matching-pairs (tasks-with-matching-location top-level-name))
         (tasks (mapcar (alexandria:rcurry #'alexandria:assoc-value '?task) (car matching-pairs)))
         (paths (mapcar #'cpl:task-tree-node-path tasks))
         (tray-poses '(((1.3469725290934245d0 0.1027636495729287465d0 0.9610342152913412d0)
                        (0 0 1 0))
                       ((1.34 0.0 1.0)
                        (0 0 0.7071 0.7071))
                       ((1.34 0.0 0.96)
                        (0 0 1 0))))
         (tray-action (tray-transporting-action)))

    (labels ((change-loc-to-tray (action-desig)
               (let ((?tray-pose (cl-transforms-stamped:pose->pose-stamped
                                  "map" 0.0
                                  (btr:ensure-pose (pop tray-poses))))
                     (?desig-cpy (desig:description (desig:copy-designator action-desig))))
                 (setf ?desig-cpy (remove (assoc :target ?desig-cpy) ?desig-cpy))
                 (push `(:target ,(desig:a location
                                           (pose ?tray-pose))) ?desig-cpy)
                 (desig:make-designator :action
                                        ?desig-cpy)))
             (btr-obj (name)
               (btr:object btr:*current-bullet-world* name)))
      (let ((transports-list
              (map 'list (lambda (path)()
                           (let* ((fetch-desig
                                    (action-designator-under-path path :fetching top-level-name))
                                  (deliver-desig
                                    (change-loc-to-tray
                                     (action-designator-under-path path :delivering top-level-name))))
                             (list fetch-desig
                                   deliver-desig
                                   (desig:desig-prop-value
                                    (desig:desig-prop-value deliver-desig :object) :name)))) paths)))
        (flet ((trans-fun (&rest desig)
                 (declare (ignore desig))
                 (dolist (transport (reverse transports-list))
                   (exe:perform (first transport))
                   (exe:perform (second transport))
                   (btr::attach-item (third transport) (btr-obj :tray-1)))
                 (exe:perform tray-action)
                 (btr::detach-all-items (btr-obj :tray-1))))

          (cpl-impl::replace-task-code '(TRANSFORM1)
                                       #'trans-fun
                                       (second paths)
                                       (cpl-impl::get-top-level-task-tree top-level-name))

          (cpl-impl::replace-task-code '(TRANSFORM2)
                                       #'(lambda (&rest desig)
                                           (declare (ignore desig)))
                                       (first paths)
                                       (cpl-impl::get-top-level-task-tree top-level-name))))))))



(defun environment-rule (&optional (top-level-name :top-level)))
