{
  "id" : 1414403413451,
  "name" : "AuthorityTest",
  "comment" : "",
  "relativeDirectory" : "Authority",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.0,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1414403429951,
    "name" : "1414403429951",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 1,
    "maxCardinality" : 1,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1414403429950,
    "plan" : 1414403413451
  }, {
    "id" : 1414403522424,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 1,
    "maxCardinality" : 1,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1414403553717,
    "plan" : 1414403413451
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1414403429950,
    "name" : "UpperState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1414403413451,
    "confAbstractPlanWrappers" : [ {
      "id" : 1626437211225,
      "name" : "1626437211225",
      "comment" : "",
      "abstractPlan" : "Behaviour/EmptyBehaviour.beh#1625610857563",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1414403553717,
    "name" : "LowerState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1414403413451,
    "confAbstractPlanWrappers" : [ {
      "id" : 1626437213779,
      "name" : "1626437213779",
      "comment" : "",
      "abstractPlan" : "Behaviour/EmptyBehaviour.beh#1625610857563",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}