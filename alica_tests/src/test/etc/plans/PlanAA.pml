{
  "id" : 1629895864090,
  "name" : "PlanAA",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1629896015785,
    "name" : "1629896015785",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613371619454",
    "state" : 1629896006533,
    "plan" : 1629895864090
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1629896006533,
    "name" : "BehAAA",
    "comment" : "",
    "entryPoint" : 1629896015785,
    "parentPlan" : 1629895864090,
    "confAbstractPlanWrappers" : [ {
      "id" : 1629896034213,
      "name" : "1629896034213",
      "comment" : "",
      "abstractPlan" : "Behaviour/BehAAA.beh#1629895901559",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}