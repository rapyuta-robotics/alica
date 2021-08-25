{
  "id" : 1629895837159,
  "name" : "PlanA",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1629895952886,
    "name" : "1629895952886",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613371619454",
    "state" : 1629895956631,
    "plan" : 1629895837159
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1629895956631,
    "name" : "PlanAA",
    "comment" : "",
    "entryPoint" : 1629895952886,
    "parentPlan" : 1629895837159,
    "confAbstractPlanWrappers" : [ {
      "id" : 1629895990979,
      "name" : "1629895990979",
      "comment" : "",
      "abstractPlan" : "PlanAA.pml#1629895864090",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}