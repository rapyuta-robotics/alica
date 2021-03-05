{
  "id" : 1613378433623,
  "name" : "SchedulingTestPlan3",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1613378578364,
    "name" : "1613378578364",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613372009777",
    "state" : 1613378580338,
    "plan" : 1613378433623
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1613378580338,
    "name" : "InitPlan3",
    "comment" : "",
    "entryPoint" : 1613378578364,
    "parentPlan" : 1613378433623,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}