{
  "id" : 1418825395939,
  "name" : "MasterSyncTransition",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1418825395941,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 1,
    "maxCardinality" : 10000,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1418825395940,
    "plan" : 1418825395939
  }, {
    "id" : 1418825402617,
    "name" : "DefaultTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 1,
    "maxCardinality" : 10000,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1418825404963,
    "plan" : 1418825395939
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1418825395940,
    "name" : "FirstTaskFirstState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418825395939,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1418825425833 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1418825404963,
    "name" : "SecondTaskFirstState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418825395939,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1418825427469 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1418825409988,
    "name" : "FirstTaskSecondState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418825395939,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662770,
      "name" : "1587718662770",
      "comment" : "",
      "abstractPlan" : "Behaviour/Attack.beh#1402488848841",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1418825425833 ]
  }, {
    "type" : "State",
    "id" : 1418825411686,
    "name" : "SecondTaskSecondState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1418825395939,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662773,
      "name" : "1587718662773",
      "comment" : "",
      "abstractPlan" : "Behaviour/Attack.beh#1402488848841",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1418825427469 ]
  } ],
  "transitions" : [ {
    "id" : 1418825425833,
    "name" : "FirstTaskTran",
    "comment" : "",
    "inState" : 1418825395940,
    "outState" : 1418825409988,
    "preCondition" : {
      "id" : 1418825427317,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : 1418825482116
  }, {
    "id" : 1418825427469,
    "name" : "SecondTaskTran",
    "comment" : "",
    "inState" : 1418825404963,
    "outState" : 1418825411686,
    "preCondition" : {
      "id" : 1418825428924,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : 1418825482116
  } ],
  "synchronisations" : [ {
    "id" : 1418825482116,
    "name" : "Sync",
    "comment" : "",
    "talkTimeout" : 30,
    "syncTimeout" : 10000,
    "failOnSyncTimeout" : false,
    "plan" : 1418825395939,
    "syncedTransitions" : [ 1418825425833, 1418825427469 ]
  } ]
}