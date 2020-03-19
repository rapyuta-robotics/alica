{
  "id" : 1414403396328,
  "name" : "AuthorityTestMaster",
  "comment" : "",
  "relativeDirectory" : "Authority",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1414403396331,
    "name" : "1414403396331",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1414403820806,
    "plan" : 1414403396328
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1414403396329,
    "name" : "testState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1414403396328,
    "abstractPlans" : [ "Authority/AuthorityTest.pml#1414403413451" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1414403840950 ]
  }, {
    "type" : "State",
    "id" : 1414403820806,
    "name" : "Init",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1414403396328,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1414403840950 ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ {
    "id" : 1414403840950,
    "name" : "1414403840950",
    "comment" : "",
    "inState" : 1414403820806,
    "outState" : 1414403396329,
    "preCondition" : {
      "id" : 1414403842622,
      "name" : "1414403842622",
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