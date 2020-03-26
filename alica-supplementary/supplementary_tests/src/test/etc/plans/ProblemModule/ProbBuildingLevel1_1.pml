{
  "id" : 1479557664989,
  "name" : "ProbBuildingLevel1_1",
  "comment" : "",
  "relativeDirectory" : "ProblemModule",
  "variables" : [ {
    "id" : 1479718387346,
    "name" : "PBL1_1X",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1479718398960,
    "name" : "PBL1_1Y",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1479557690963,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1479557690962,
    "plan" : 1479557664989
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1479557690962,
    "name" : "MiddleState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1479557664989,
    "abstractPlans" : [ "ProblemModule/QueryPlan2.pml#1479718449392" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}