{
  "id" : 1625614640417,
  "name" : "EngineRulesSchedulingTestPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1625614705483,
    "name" : "1625614705483",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1625610762033",
    "state" : 1625614714499,
    "plan" : 1625614640417
  }, {
    "id" : 1625614710816,
    "name" : "1625614710816",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1625610785404",
    "state" : 1625614697742,
    "plan" : 1625614640417
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1625614697742,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : 1625614710816,
    "parentPlan" : 1625614640417,
    "confAbstractPlanWrappers" : [ {
      "id" : 1625644679751,
      "name" : "1625644679751",
      "comment" : "",
      "abstractPlan" : "Behaviour/EmptyBehaviour.beh#1625610857563",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1625614714499,
    "name" : "StartEngineRulesSchedulingTest",
    "comment" : "",
    "entryPoint" : 1625614705483,
    "parentPlan" : 1625614640417,
    "confAbstractPlanWrappers" : [ {
      "id" : 1625781179215,
      "name" : "1625781179215",
      "comment" : "",
      "abstractPlan" : "Behaviour/EmptyBehaviour.beh#1625610857563",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1625614729978, 1625776897471 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1625614719367,
    "name" : "Default Name",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1625614640417,
    "confAbstractPlanWrappers" : [ {
      "id" : 1625644677437,
      "name" : "1625644677437",
      "comment" : "",
      "abstractPlan" : "Behaviour/EmptyBehaviour.beh#1625610857563",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1625614729978 ]
  }, {
    "type" : "TerminalState",
    "id" : 1625776883489,
    "name" : "FailureState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1625614640417,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1625776897471 ],
    "success" : false,
    "postCondition" : null
  } ],
  "transitions" : [ {
    "id" : 1625614729978,
    "name" : "FromDefault NameTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1625614714499,
    "outState" : 1625614719367,
    "preCondition" : {
      "id" : 1625614729981,
      "name" : "1625614729981",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1625776897471,
    "name" : "FromStartEngineRulesSchedulingTestTo Default Name",
    "comment" : "MISSING_COMMENT",
    "inState" : 1625614714499,
    "outState" : 1625776883489,
    "preCondition" : {
      "id" : 1625776897472,
      "name" : "1625776897472",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  } ],
  "synchronisations" : [ ]
}