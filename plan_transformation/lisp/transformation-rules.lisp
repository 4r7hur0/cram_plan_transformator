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
  (let ((pick-list (find-task '(cram-pr2-pick-place-plans:pick-up) tltt)))
    (if (equal (get-robot-position (first pick-list)) (get-robot-position (second pick-list)))
        (replace-task-code '(BRING-AT-ONCE) #'bring-at-once
                           '((BRING-ONE-BY-ONE) (CRAM-LANGUAGE-IMPLEMENTATION:TOP-LEVEL TT2)) tltt)
        (warn "rule not applicable"))))

(defun apply-double-grab (tltt)
  (let ((pick-list (find-task '(cram-pr2-pick-place-plans:pick-up) tltt)))
    (if (equal (get-robot-position (first pick-list)) (get-robot-position (second pick-list)))
        (progn (replace-task-code '(PICK-TWO)
                                  (lambda (?object-designator (&optional (?arm (first )
                             ;; (pick-up (second (cpl-impl:code-parameters (cpl:task-tree-node-code (first pick-list)))))
                             ;; (pick-up (second (cpl-impl:code-parameters (cpl:task-tree-node-code (second pick-list)))) :right)
                             
                           '((PICK-UP) (FETCH-BOTTLE-FROM-COUNTER) (BRING-ONE-BY-ONE) (CRAM-LANGUAGE-IMPLEMENTATION:TOP-LEVEL TT2))
                           ;; (intern (write-to-string (cpl:task-tree-node-path (cpl:task-tree-node-parent (second pick-list)))))
                           tltt)
               (replace-task-code '(print) (lambda (&optional x) (warn "oi") (warn "~a" x))
                                  '((FETCH-BOTTLE-FROM-COUNTER :CALL 2)
                                    (BRING-ONE-BY-ONE) (CRAM-LANGUAGE-IMPLEMENTATION:TOP-LEVEL TT2)) tltt))
        (warn "apply-double-grab not applicable"))))
