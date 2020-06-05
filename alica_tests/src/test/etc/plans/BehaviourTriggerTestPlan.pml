{
  "id" : 1428508768572,
  "name" : "BehaviourTriggerTestPlan",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1428508768574,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1428508768573,
    "plan" : 1428508768572
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1428508768573,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1428508768572,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662751,
      "name" : "1587718662751",
      "comment" : "",
      "abstractPlan" : "Behaviour/TriggerA.beh#1428508297492",
      "configuration" : null
    }, {
      "id" : 1587718662753,
      "name" : "1587718662753",
      "comment" : "",
      "abstractPlan" : "Behaviour/TriggerB.beh#1428508316905",
      "configuration" : null
    }, {
      "id" : 1587718662755,
      "name" : "1587718662755",
      "comment" : "",
      "abstractPlan" : "Behaviour/TriggerC.beh#1428508355209",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1429017235181 ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1429017227839,
    "name" : "NewState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1428508768572,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662758,
      "name" : "1587718662758",
      "comment" : "",
      "abstractPlan" : "Behaviour/NotToTrigger.beh#1429017274116",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ 1429017235181 ]
  } ],
  "transitions" : [ {
    "id" : 1429017235181,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1428508768573,
    "outState" : 1429017227839,
    "preCondition" : {
      "id" : 1429017236633,
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