{
  "id" : 1522377375148,
  "name" : "BehaviorSuccessSpamMaster",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1522377375150,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1522377375149,
    "plan" : 1522377375148
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1522377375149,
    "name" : "Normal",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1522377375148,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718663016,
      "name" : "1587718663016",
      "comment" : "",
      "abstractPlan" : "Behaviour/SuccessSpam.beh#1522377401286",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1522377944058 ],
    "inTransitions" : [ 1522377945069 ]
  }, {
    "type" : "State",
    "id" : 1522377929290,
    "name" : "Dummy",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1522377375148,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718663020,
      "name" : "1587718663020",
      "comment" : "",
      "abstractPlan" : "Behaviour/SuccessSpam.beh#1522377401286",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1522377945069 ],
    "inTransitions" : [ 1522377944058 ]
  } ],
  "transitions" : [ {
    "id" : 1522377944058,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1522377375149,
    "outState" : 1522377929290,
    "preCondition" : {
      "id" : 1522377944921,
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
    "id" : 1522377945069,
    "name" : "MISSING_NAME",
    "comment" : "",
    "inState" : 1522377929290,
    "outState" : 1522377375149,
    "preCondition" : {
      "id" : 1522377946607,
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