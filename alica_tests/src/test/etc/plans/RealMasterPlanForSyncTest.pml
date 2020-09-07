{
  "id" : 1418902217839,
  "name" : "RealMasterPlanForSyncTest",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1418902217841,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1418902217840,
    "plan" : 1418902217839
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1418902217840,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418902217839,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662652,
      "name" : "1587718662652",
      "comment" : "",
      "abstractPlan" : "MasterSyncTransition.pml#1418825395939",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}