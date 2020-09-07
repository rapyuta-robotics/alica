{
  "id" : 1413200862180,
  "name" : "MultiAgentTestPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1413200877337,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : true,
    "minCardinality" : 1,
    "maxCardinality" : 1,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1413200877336,
    "plan" : 1413200862180
  }, {
    "id" : 1413200890537,
    "name" : "DefaultTask",
    "comment" : "",
    "successRequired" : true,
    "minCardinality" : 1,
    "maxCardinality" : 1,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1413200910490,
    "plan" : 1413200862180
  }, {
    "id" : 1413807260446,
    "name" : "NewEntryPoint",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1413807264574,
    "plan" : 1413200862180
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1413200877336,
    "name" : "OtherState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200862180,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662849,
      "name" : "1587718662849",
      "comment" : "",
      "abstractPlan" : "Behaviour/Attack.beh#1402488848841",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1413201368286 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1413200910490,
    "name" : "State1",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200862180,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662852,
      "name" : "1587718662852",
      "comment" : "",
      "abstractPlan" : "Behaviour/Attack.beh#1402488848841",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1413201050743 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1413201030936,
    "name" : "State2",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200862180,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662855,
      "name" : "1587718662855",
      "comment" : "",
      "abstractPlan" : "Behaviour/Attack.beh#1402488848841",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1413201367062 ],
    "inTransitions" : [ 1413201050743 ]
  }, {
    "type" : "TerminalState",
    "id" : 1413201164999,
    "name" : "NewSuccessState1",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200862180,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1413201368286 ],
    "success" : true,
    "postCondition" : null
  }, {
    "type" : "TerminalState",
    "id" : 1413552736921,
    "name" : "NewSuccessState2",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200862180,
    "confAbstractPlanWrappers" : [ ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1413201367062 ],
    "success" : true,
    "postCondition" : null
  }, {
    "type" : "State",
    "id" : 1413807264574,
    "name" : "Idle",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1413200862180,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662860,
      "name" : "1587718662860",
      "comment" : "",
      "abstractPlan" : "Behaviour/AttackOpp.beh#1402489351885",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ {
    "id" : 1413201050743,
    "name" : "1413201050743",
    "comment" : "",
    "inState" : 1413200910490,
    "outState" : 1413201030936,
    "preCondition" : {
      "id" : 1413201052549,
      "name" : "1413201052549",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1413201367062,
    "name" : "1413201367062",
    "comment" : "",
    "inState" : 1413201030936,
    "outState" : 1413552736921,
    "preCondition" : {
      "id" : 1413201367990,
      "name" : "1413201367990",
      "comment" : "",
      "enabled" : true,
      "conditionString" : "",
      "pluginName" : "DefaultPlugin",
      "variables" : [ ],
      "quantifiers" : [ ]
    },
    "synchronisation" : null
  }, {
    "id" : 1413201368286,
    "name" : "1413201368286",
    "comment" : "",
    "inState" : 1413200877336,
    "outState" : 1413201164999,
    "preCondition" : {
      "id" : 1413201370590,
      "name" : "1413201370590",
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