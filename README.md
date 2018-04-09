# cram_plan_transformation
This module implements simple plans written in CRAM-language, that can be obducted in the CRAM-task-tree. I want to showcase the optimization of plan-based robotics through plan-transformation.

## Installation

* Create a catkin workspace and clone this repo into it
* Create two additional workspaces, one for knowrob, the other for CRAM dependencies 
and initialize wstool within its src directory.
* Merge cram.rosinstall and knowrob.rosinstall into their wstool workspaces
* First pull and catkin_make knowrob, then CRAM. Probably you will get compilation errors with knowrob on your first try.
Just run it again and they will disappear.
* Source those workspaces' devel/setup.bash and compile the cram_plan_transformation workspace

## Usage

There are three scenarios and three transformations to try out. Load the *plan_transformation* package in your REPL and change your package to *plt*, the nickname. Now run *(roslisp-utilities:startup-ros)* to initialize the environment and start the simulation. This might take a moment, depending on your machine around 2 to 5 minutes.

To test the scenarios you can call following functions, defined in  *top-level.lisp*:
* *(test-collect-from-sink)* starts collecting 4 items from the sink area.
* *(test-collect-from-fridge)* starts collecting 3 items from the fridge.
* *(test-collect-mixed)* collects 2 items from the sink area and 2 from the fridge.

After a test has run you can investigate the task tree with `(cpl:get-top-level-task-tree *top-level-name*)`. Make sure to change the `*top-level-name*` parameter if you use custom top level names, it is `:top-level` per default. If you want to start with checking applicability of rules you can look into *predicates.lisp*, at the bottom of the file you will find the transformation specific predicates. The transformation functions are in *transformation-rules.lisp*. Here you can also see, that the transformation rules are registered to the generic transformation framework, with their respective applicability predicate.

An example usage of transforming a plan could look like this
```
(test-collect-from-sink) ;; to create the task tree, our target of transformation.
(apply-rules) ;; will check applicability of all registered rules and apply the first one found suitable.
;; A message will tell, which rule has been applied.
(test-collect-from-sink NIL) ;; per default this function resets the task tree. To prevent that, add NIL as parameter.
```
If you want to tweak the transformation rules, you can add new ones, disable them or set priorities.
```
;; Register the both-hands-rule and stacking-rule with their predicates
(register-transformation-rule both-hands-transporting-rule
                              '(task-transporting-siblings ?first-transport ?second-transport))
(register-transformation-rule stacking-rule
                              '(task-transporting-with-tray ?last-path))

;; Prioritize stacking rule before both-hands-rule
(prioritize-rule 'stacking-rule 'both-hands-transporting-rule)

;; Disable and enale environent-rule
(disable-transformation-rule 'environment-rule)
(enable-transformation-rule 'environment-rule)
```

Check the parameters `*transformation-rules*`, `*disabled-rules*` and `*rule-priority*` if you lose track of your operations. When everything is at your liking, and there is a task tree to manipulate, you can always run `(apply-rules)` and run the test function again.
