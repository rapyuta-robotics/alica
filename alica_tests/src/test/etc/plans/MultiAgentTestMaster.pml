{
  "id" : 1413200842973,
  "name" : "MultiAgentTestMaster",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1413200842975,
    "name" : "1413200842975",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1413200842974,
    "plan" : 1413200842973
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1413200842974,
    "name" : "Init",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200842973,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1413201226246 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1413201213955,
    "name" : "Start",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200842973,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662949,
      "name" : "1587718662949",
      "comment" : "",
      "abstractPlan" : "MultiAgentTestPlan.pml#1413200862180",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1413201388722 ],
    "inTransitions" : [ 1413201226246 ]
  }, {
    "type" : "State",
    "id" : 1413201380359,
    "name" : "Finished",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200842973,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662952,
      "name" : "1587718662952",
      "comment" : "",
      "abstractPlan" : "Behaviour/Attack.beh#1402488848841",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1413201388722 ]
  } ],
  "transitions" : [ {
    "id" : 1413201226246,
    "name" : "1413201226246",
    "comment" : "",
    "inState" : 1413200842974,
    "outState" : 1413201213955,
    "preCondition" : {
      "id" : 1413201227586,
      "name" : "1413201227586",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1413201388722,
    "name" : "1413201388722",
    "comment" : "",
    "inState" : 1413201213955,
    "outState" : 1413201380359,
    "preCondition" : {
      "id" : 1413201389955,
      "name" : "1413201389955",
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