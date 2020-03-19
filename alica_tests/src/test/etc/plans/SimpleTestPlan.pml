{
  "id" : 1412252439925,
  "name" : "SimpleTestPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : {
    "id" : 1412781707952,
    "name" : "NewPreCondition",
    "comment" : "",
    "enabled" : true,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ ],
    "quantifiers" : [ ]
  },
  "runtimeCondition" : {
    "id" : 1412781693884,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ ],
    "quantifiers" : [ ]
  },
  "entryPoints" : [ {
    "id" : 1412252439927,
    "name" : "1412252439927",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1412252439926,
    "plan" : 1412252439925
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1412252439926,
    "name" : "TestState1",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1412252439925,
    "abstractPlans" : [ "Behaviour/MidFieldStandard.beh#1402488696205" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1412761925032 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1412761855746,
    "name" : "TestState2",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1412252439925,
    "abstractPlans" : [ "Behaviour/Attack.beh#1402488848841" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1412761925032 ]
  } ],
  "transitions" : [ {
    "id" : 1412761925032,
    "name" : "1412761925032",
    "comment" : "",
    "inState" : 1412252439926,
    "outState" : 1412761855746,
    "preCondition" : {
      "id" : 1412761926856,
      "name" : "1412761926856",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}