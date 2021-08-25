{
  "id" : 1629895873188,
  "name" : "PlanBA",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1629896091322,
    "name" : "1629896091322",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613371619454",
    "state" : 1629896094706,
    "plan" : 1629895873188
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1629896094706,
    "name" : "BehBAA",
    "comment" : "",
    "entryPoint" : 1629896091322,
    "parentPlan" : 1629895873188,
    "confAbstractPlanWrappers" : [ {
      "id" : 1629896123291,
      "name" : "1629896123291",
      "comment" : "",
      "abstractPlan" : "Behaviour/BehBAA.beh#1629895911592",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}