(in-package :plt-tests)

(defun generate-data (input-data)
  (top-level-root input-data))

(def-top-level-cram-function top-level-root (test-data-top-level)
  (first-node test-data-top-level)
  (second-node test-data-top-level))

(def-cram-function first-node (test-data-first-node)
  (format t test-data-first-node))

(def-cram-function second-node (test-data-second-node)
  (third-node test-data-second-node))

(def-cram-function third-node (test-data-third-node)
  (format t (concatenate 'string test-data-third-node " teststring")))
