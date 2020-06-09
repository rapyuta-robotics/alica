{
  "id" : 1479718449392,
  "name" : "QueryPlan2",
  "comment" : "",
  "relativeDirectory" : "ProblemModule",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1479718449394,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1479718449393,
    "plan" : 1479718449392
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1479718449393,
    "name" : "QueryState2",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1479718449392,
    "abstractPlans" : [ "ProblemModule/QueryBehaviour1.beh#1479556104511" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}