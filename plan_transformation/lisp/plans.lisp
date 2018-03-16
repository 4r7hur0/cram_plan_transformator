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

(cpl:def-cram-function access-container (?target-location
                                         &optional (?joint-name "iai_fridge_door_joint"))
  (cpl:with-failure-handling
        ((common-fail:high-level-failure (e)
           (declare (ignore e))
           (roslisp:ros-warn (pp-plans access-container)
                             "Navigating to container impossible at pose~%~a."
                             (desig:desig-prop-value ?target-location :pose))))
           (exe:perform (desig:an action
                                  (type navigating)
                                  (location ?target-location))))
  (btr:set-robot-state-from-joints
   `((,?joint-name 2.5))
   (btr:object btr:*current-bullet-world* :kitchen))
  (btr:detach-all-objects (btr:object btr:*current-bullet-world* :kitchen))
  (setf pr2-proj-reasoning::*allow-pick-up-kitchen-collision* t))


(cpl:def-cram-function close-container (?target-location
                                        &optional (?joint-name "iai_fridge_door_joint"))
  (setf pr2-proj-reasoning::*allow-pick-up-kitchen-collision* nil)
  (cpl:with-failure-handling
        ((common-fail:high-level-failure (e)
           (declare (ignore e))
           (roslisp:ros-warn (pp-plans access-container)
                             "Navigating to container impossible at pose~%~a."
                             (desig:desig-prop-value ?target-location :pose))))
           (exe:perform (desig:an action
                                  (type navigating)
                                  (location ?target-location))))
  (btr:set-robot-state-from-joints
   `((,?joint-name 0.0))
   (btr:object btr:*current-bullet-world* :kitchen))
  (btr:detach-all-objects (btr:object btr:*current-bullet-world* :kitchen)))


(cpl:def-cram-function transport-from-container (?container-location
                                                 ?object-designator
                                                 ?fetching-location
                                                 ?delivering-location
                                                 ?arm
                                                 &optional
                                                 (?retract-arms t)
                                                 (?joint-name "iai_fridge_door_joint"))
    
  (exe:perform 
     (desig:an action
               (type accessing-container)
               (location ?container-location)))
  (cpl:with-failure-handling
      (((or common-fail:object-unfetchable
            common-fail:high-level-failure) (e)
           (declare (ignore e))
           (roslisp:ros-warn (pp-plans access-container)
                             "Fetching was not successfull for object: ~a~%. Closing the container~%."
                             (desig:desig-prop-value ?object-designator :name))
           (exe:perform 
            (desig:an action
                      (type closing-container)
                      (location ?container-location)))))
    (let ((?fetched-object (exe:perform
                            (desig:an action
                                      (type fetching)
                                      (when ?arm
                                        (arm ?arm))
                                      (object ?object-designator)
                                      (location ?fetching-location)
                                      (retract-arms ?retract-arms)))))
    (exe:perform 
     (desig:an action
               (type closing-container)
               (location ?container-location)))
    (cpl:with-failure-handling
        ((common-fail:high-level-failure (e)
           (declare (ignore e))
           (pr2-fd-plans::drop-at-sink)))
      (exe:perform (desig:an action
                             (type delivering)
                             (object ?fetched-object)
                             (target ?delivering-location)
                             (retract-arms ?retract-arms)))))))
