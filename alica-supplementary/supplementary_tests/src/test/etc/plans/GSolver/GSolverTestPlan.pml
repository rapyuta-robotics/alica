{
  "id" : 1417423757243,
  "name" : "GSolverTestPlan",
  "comment" : "",
  "relativeDirectory" : "GSolver",
  "variables" : [ {
    "id" : 1417444589341,
    "name" : "X",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1417444593509,
    "name" : "Y",
    "comment" : "",
    "variableType" : ""
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1417424512343,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ 1417444589341, 1417444593509 ],
    "quantifiers" : [ ]
  },
  "entryPoints" : [ {
    "id" : 1417423777546,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1417423777544,
    "plan" : 1417423757243
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1417423777544,
    "name" : "SolverState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1417423757243,
    "abstractPlans" : [ "GSolver/SolverTestBehaviour.beh#1417424455986" ],
    "variableBindings" : [ {
      "id" : 1417444715623,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1417444593509,
      "subPlan" : "GSolver/SolverTestBehaviour.beh#1417424455986",
      "subVariable" : "GSolver/SolverTestBehaviour.beh#1417444710642"
    }, {
      "id" : 1417444720874,
      "name" : "MISSING_NAME",
      "comment" : "",
      "variable" : 1417444589341,
      "subPlan" : "GSolver/SolverTestBehaviour.beh#1417424455986",
      "subVariable" : "GSolver/SolverTestBehaviour.beh#1417444707321"
    } ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}