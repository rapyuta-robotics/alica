{
  "id" : 1629895853508,
  "name" : "PlanB",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1629896055805,
    "name" : "1629896055805",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613371619454",
    "state" : 1629896057548,
    "plan" : 1629895853508
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1629896057548,
    "name" : "PlanBA",
    "comment" : "",
    "entryPoint" : 1629896055805,
    "parentPlan" : 1629895853508,
    "confAbstractPlanWrappers" : [ {
      "id" : 1629896077656,
      "name" : "1629896077656",
      "comment" : "",
      "abstractPlan" : "PlanBA.pml#1629895873188",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}