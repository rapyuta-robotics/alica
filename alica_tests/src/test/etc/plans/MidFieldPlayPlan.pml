{
  "id" : 1402488770050,
  "name" : "MidFieldPlayPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : {
    "id" : 1402489260911,
    "name" : "NewRuntimeCondition",
    "comment" : "",
    "enabled" : false,
    "conditionString" : "",
    "pluginName" : "DefaultPlugin",
    "variables" : [ ],
    "quantifiers" : [ ]
  },
  "entryPoints" : [ {
    "id" : 1402488787819,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : true,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1402488787818,
    "plan" : 1402488770050
  }, {
    "id" : 1402500828244,
    "name" : "NewEntryPoint",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 3,
    "maxCardinality" : 5,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1402500830885,
    "plan" : 1402488770050
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1402488787818,
    "name" : "Wander",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488770050,
    "abstractPlans" : [ "Behaviour/MidFieldStandard.beh#1402488696205" ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402489257607, 1402489276995 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1402489237914,
    "name" : "Tackle",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488770050,
    "abstractPlans" : [ "Tackle.pml#1402489318663", "Behaviour/Attack.beh#1402488848841", "Defend.pml#1402488893641" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1402489257607 ]
  }, {
    "type" : "State",
    "id" : 1402489273401,
    "name" : "Sync",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488770050,
    "abstractPlans" : [ "Behaviour/Tackle.beh#1402488939130" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1402489276995 ]
  }, {
    "type" : "State",
    "id" : 1402500830885,
    "name" : "Kill",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488770050,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402500843072 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1402500833246,
    "name" : "Shoot",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488770050,
    "abstractPlans" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1402500843072 ]
  } ],
  "transitions" : [ {
    "id" : 1402489257607,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402488787818,
    "outState" : 1402489237914,
    "preCondition" : {
      "id" : 1402489258509,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1402489276995,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402488787818,
    "outState" : 1402489273401,
    "preCondition" : {
      "id" : 1402489278408,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : 1402500865502
  }, {
    "id" : 1402500843072,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1402500830885,
    "outState" : 1402500833246,
    "preCondition" : {
      "id" : 1402500844446,
      "name" : "MISSING_NAME",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : 1402500865502
  } ],
  "synchronisations" : [ {
    "id" : 1402500865502,
    "name" : "SynChro",
    "comment" : "",
    "talkTimeout" : 30,
    "syncTimeout" : 10000,
    "failOnSyncTimeout" : false,
    "plan" : 1402488770050,
    "syncedTransitions" : [ 1402500843072, 1402489276995 ]
  } ]
}