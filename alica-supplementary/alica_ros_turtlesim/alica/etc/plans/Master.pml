{
  "id" : 1542881176278,
  "name" : "Master",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1543227864154,
    "name" : "NewEntryPoint",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "TaskRepository.tsk#1542881176318",
    "state" : 1542881176280,
    "plan" : 1542881176278
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1542881176280,
    "name" : "Init",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1542881176278,
    "confAbstractPlanWrappers" : [ {
      "id" : 1601278930769,
      "name" : "1601278930769",
      "comment" : "",
      "abstractPlan" : "Behaviours/Go2RandomPosition.beh#1542881969548",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1542881645594 ],
    "inTransitions" : [ 1542881648973 ]
  }, {
    "type" : "State",
    "id" : 1542881580237,
    "name" : "Move",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1542881176278,
    "confAbstractPlanWrappers" : [ {
      "id" : 1601278930772,
      "name" : "1601278930772",
      "comment" : "",
      "abstractPlan" : "Move.pml#1542882005838",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ 1542881648973 ],
    "inTransitions" : [ 1542881645594 ]
  } ],
  "transitions" : [ {
    "id" : 1542881645594,
    "name" : "Init2Move",
    "comment" : "",
    "inState" : 1542881176280,
    "outState" : 1542881580237,
    "preCondition" : {
      "id" : 1542881647180,
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
    "id" : 1542881648973,
    "name" : "Move2Init",
    "comment" : "Transition from Move to Init",
    "inState" : 1542881580237,
    "outState" : 1542881176280,
    "preCondition" : {
      "id" : 1542881650423,
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
