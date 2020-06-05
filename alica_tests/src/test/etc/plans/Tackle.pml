{
  "id" : 1402489318663,
  "name" : "Tackle",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1402489329142,
    "name" : "MISSING_NAME",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1402489329141,
    "plan" : 1402489318663
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1402489329141,
    "name" : "AttackOpp",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1402489318663,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662873,
      "name" : "1587718662873",
      "comment" : "",
      "abstractPlan" : "Behaviour/AttackOpp.beh#1402489351885",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}