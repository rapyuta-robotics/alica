{
  "id" : 1479556074049,
  "name" : "QueryPlan1",
  "comment" : "",
  "relativeDirectory" : "ProblemModule",
  "variables" : [ {
    "id" : 1479556220234,
    "name" : "QP1X",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1479556572534,
    "name" : "QP1Y",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1479556084493,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1479556220234, 1479556572534 ],
    "quantifiers" : [ {
      "id" : 1479556181307,
      "name" : "MISSING_NAME",
      "comment" : "",
      "quantifierType" : "all",
      "scope" : 1479556074050,
      "sorts" : [ "X", "Y", "Z" ]
    } ]
  },
  "entryPoints" : [ {
    "id" : 1479556074051,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1479556074050,
    "plan" : 1479556074049
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1479556074050,
    "name" : "QueryState1",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1479556074049,
    "confAbstractPlanWrappers" : [ {
      "id" : 1597658636505,
      "name" : "1597658636505",
      "comment" : "",
      "abstractPlan" : "ProblemModule/QueryBehaviour1.beh#1479556104511",
      "configuration" : null
    } ],
    "variableBindings" : [ {
      "id" : 1479557255352,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1479556220234,
      "subPlan" : "ProblemModule/QueryBehaviour1.beh#1479556104511",
      "subVariable" : "ProblemModule/QueryBehaviour1.beh#1479556246733"
    }, {
      "id" : 1479557270121,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1479556572534,
      "subPlan" : "ProblemModule/QueryBehaviour1.beh#1479556104511",
      "subVariable" : "ProblemModule/QueryBehaviour1.beh#1479557263650"
    } ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}
