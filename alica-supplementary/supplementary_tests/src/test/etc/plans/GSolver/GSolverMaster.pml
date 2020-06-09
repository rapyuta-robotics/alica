{
  "id" : 1417423751087,
  "name" : "GSolverMaster",
  "comment" : "",
  "relativeDirectory" : "GSolver",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1417423751089,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1417423751088,
    "plan" : 1417423751087
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1417423751088,
    "name" : "Init",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1417423751087,
    "abstractPlans" : [ "GSolver/GSolverTestPlan.pml#1417423757243" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}