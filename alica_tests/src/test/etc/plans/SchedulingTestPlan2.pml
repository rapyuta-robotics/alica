{
  "id" : 1613378423610,
  "name" : "SchedulingTestPlan2",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1613378624077,
    "name" : "1613378624077",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1613372009777",
    "state" : 1613378625757,
    "plan" : 1613378423610
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1613378625757,
    "name" : "InitPlan2",
    "comment" : "",
    "entryPoint" : 1613378624077,
    "parentPlan" : 1613378423610,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}