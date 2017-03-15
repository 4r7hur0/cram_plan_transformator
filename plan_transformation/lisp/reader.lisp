(in-package :plt)

(defun read-from-task-tree (tree-name)
  "Reads from task tree and returns it.
TREE-NAME: Name of the task tree."
  (cpl-impl:get-top-level-task-tree tree-name))


