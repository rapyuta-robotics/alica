{
  "id" : 1402488437260,
  "name" : "MasterPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1402488437263,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1402488437261,
    "plan" : 1402488437260
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1402488437261,
    "name" : "Attack",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488437260,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662710,
      "name" : "1587718662710",
      "comment" : "",
      "abstractPlan" : "Behaviour/Attack.beh#1402488848841",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402488517667, 1409218318661 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1402488463437,
    "name" : "Defend",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488437260,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662713,
      "name" : "1587718662713",
      "comment" : "",
      "abstractPlan" : "Defend.pml#1402488893641",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1409218318661 ]
  }, {
    "type" : "State",
    "id" : 1402488470615,
    "name" : "Goal",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488437260,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662716,
      "name" : "1587718662716",
      "comment" : "",
      "abstractPlan" : "GoalPlan.pml#1402488870347",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402488557864 ],
    "inTransitions" : [ 1402488519757 ]
  }, {
    "type" : "State",
    "id" : 1402488477650,
    "name" : "MidField",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488437260,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662719,
      "name" : "1587718662719",
      "comment" : "",
      "abstractPlan" : "Behaviour/MidFieldStandard.beh#1402488696205",
      "configuration" : null
    }, {
      "id" : 1587718662721,
      "name" : "1587718662721",
      "comment" : "",
      "abstractPlan" : "Behaviour/DefendMid.beh#1402488730695",
      "configuration" : null
    }, {
      "id" : 1587718662723,
      "name" : "1587718662723",
      "comment" : "",
      "abstractPlan" : "MidFieldPlayPlan.pml#1402488770050",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1402488519757 ],
    "inTransitions" : [ 1402488517667 ]
  }, {
    "type" : "TerminalState",
    "id" : 1402488536570,
    "name" : "SucGoalState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402488437260,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1402488557864 ],
    "success" : true,
    "postCondition" : null
  } ],
  "transitions" : [ {
    "id" : 1402488517667,
    "name" : "AttackToGoal",
    "comment" : "",
    "inState" : 1402488437261,
    "outState" : 1402488477650,
    "preCondition" : {
      "id" : 1402488519140,
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
    "id" : 1402488519757,
    "name" : "MidFieldToGoal",
    "comment" : "",
    "inState" : 1402488477650,
    "outState" : 1402488470615,
    "preCondition" : {
      "id" : 1402488520968,
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
    "id" : 1402488557864,
    "name" : "GoalToSucGoal",
    "comment" : "",
    "inState" : 1402488470615,
    "outState" : 1402488536570,
    "preCondition" : {
      "id" : 1402488558741,
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
    "id" : 1409218318661,
    "name" : "AttackToDefend",
    "comment" : "",
    "inState" : 1402488437261,
    "outState" : 1402488463437,
    "preCondition" : {
      "id" : 1409218319990,
      "name" : "MISSING_NAME",
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