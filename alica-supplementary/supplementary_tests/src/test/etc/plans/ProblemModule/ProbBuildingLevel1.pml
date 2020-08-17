{
  "id" : 1479557378264,
  "name" : "ProbBuildingLevel1",
  "comment" : "",
  "relativeDirectory" : "ProblemModule",
  "variables" : [ {
    "id" : 1479557432793,
    "name" : "PBL1X",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1479557444388,
    "name" : "PBL1Y",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1479557378266,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1479557378265,
    "plan" : 1479557378264
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1479557378265,
    "name" : "PTState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1479557378264,
    "confAbstractPlanWrappers" : [ {
      "id" : 1597658636459,
      "name" : "1597658636459",
      "comment" : "",
      "abstractPlan" : "ProblemModule/QueryPlantype.pty#1479557395790",
      "configuration" : null
    } ],
    "variableBindings" : [ {
      "id" : 1479557505697,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1479557432793,
      "subPlan" : "ProblemModule/QueryPlantype.pty#1479557395790",
      "subVariable" : "ProblemModule/QueryPlantype.pty#1479557463468"
    }, {
      "id" : 1479557512341,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1479557444388,
      "subPlan" : "ProblemModule/QueryPlantype.pty#1479557395790",
      "subVariable" : "ProblemModule/QueryPlantype.pty#1479557473424"
    } ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}