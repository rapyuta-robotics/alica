{
  "id" : 1402488634525,
  "name" : "AttackPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ {
    "id" : 1403772778288,
    "name" : "TestVar1",
    "comment" : "",
    "variableType" : "double"
  }, {
    "id" : 1403772797469,
    "name" : "VarTest2",
    "comment" : "",
    "variableType" : "int"
  }, {
    "id" : 1403772816953,
    "name" : "NewVar",
    "comment" : "",
    "variableType" : ""
  }, {
    "id" : 1403772834750,
    "name" : "ABC",
    "comment" : "",
    "variableType" : "FOL"
  } ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1402488646221,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1402488646220,
    "plan" : 1402488634525
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1402488646220,
    "name" : "Attack",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488634525,
    "abstractPlans" : [ "Tackle.pml#1402489318663", "Behaviour/AttackOpp.beh#1402489351885" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402489459382 ],
    "inTransitions" : [ 1402489460694 ]
  }, {
    "type" : "State",
    "id" : 1402489396914,
    "name" : "Shoot",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488634525,
    "abstractPlans" : [ "Behaviour/Attack.beh#1402488848841" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402489460694 ],
    "inTransitions" : [ 1402489459382 ]
  } ],
  "transitions" : [ {
    "id" : 1402489459382,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402488646220,
    "outState" : 1402489396914,
    "preCondition" : {
      "id" : 1402489460549,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ 1403772834750, 1403772778288 ],
      "quantifiers" : [ {
        "id" : 1403773214317,
        "name" : "MISSING_NAME",
        "comment" : "",
        "quantifierType" : "ALL",
        "scope" : 1402488634525,
        "sorts" : [ "X", "Y" ]
      }, {
        "id" : 1403773224776,
        "name" : "MISSING_NAME",
        "comment" : "",
        "quantifierType" : "ALL",
        "scope" : 1402488646220,
        "sorts" : [ "A", "B" ]
      }, {
        "id" : 1403773234841,
        "name" : "MISSING_NAME",
        "comment" : "",
        "quantifierType" : "ALL",
        "scope" : 1402489396914,
        "sorts" : [ "another one" ]
      }, {
        "id" : 1403773248357,
        "name" : "MISSING_NAME",
        "comment" : "",
        "quantifierType" : "ALL",
        "scope" : 1402488646221,
        "sorts" : [ "TaskQuantifier" ]
      } ]
    },
    "synchronisation" : null
  }, {
    "id" : 1402489460694,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402489396914,
    "outState" : 1402488646220,
    "preCondition" : {
      "id" : 1402489462088,
      "name" : "1402489462088",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "Some nice comment!",
      "pluginName" : "DefaultPlugin",
      "variables" : [ 1403772778288, 1403772797469, 1403772816953, 1403772834750 ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}