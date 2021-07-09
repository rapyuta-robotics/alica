{
  "id" : 1625610679488,
  "name" : "EngineRulesSchedulingTestMaster",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1625614674465,
    "name" : "1625614674465",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1625614670867",
    "state" : 1625783824098,
    "plan" : 1625610679488
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1625614677498,
    "name" : "GoIntoSubPlan",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1625610679488,
    "confAbstractPlanWrappers" : [ {
      "id" : 1625614693778,
      "name" : "1625614693778",
      "comment" : "",
      "abstractPlan" : "EngineRulesSchedulingTestPlan.pml#1625614640417",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1625783867494 ]
  }, {
    "type" : "State",
    "id" : 1625783824098,
    "name" : "EntryState",
    "comment" : "",
    "entryPoint" : 1625614674465,
    "parentPlan" : 1625610679488,
    "confAbstractPlanWrappers" : [ {
      "id" : 1625784437764,
      "name" : "1625784437764",
      "comment" : "",
      "abstractPlan" : "Behaviour/EmptyBehaviour.beh#1625610857563",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1625783867494, 1625783869824 ],
    "inTransitions" : [ ]
  }, {
    "type" : "TerminalState",
    "id" : 1625783835198,
    "name" : "FailureState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1625610679488,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1625783869824 ],
    "success" : false,
    "postCondition" : null
  } ],
  "transitions" : [ {
    "id" : 1625783867494,
    "name" : "FromEntryStateTo GoIntoSubPlan",
    "comment" : "MISSING_COMMENT",
    "inState" : 1625783824098,
    "outState" : 1625614677498,
    "preCondition" : {
      "id" : 1625783867495,
      "name" : "1625783867495",
      "comment" : "",
      "enabled" : true,
      "conditionString" : null,
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1625783869824,
    "name" : "FromEntryStateTo FailureState",
    "comment" : "MISSING_COMMENT",
    "inState" : 1625783824098,
    "outState" : 1625783835198,
    "preCondition" : {
      "id" : 1625783869825,
      "name" : "1625783869825",
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