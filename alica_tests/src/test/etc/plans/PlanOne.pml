{
  "id" : 1407153611768,
  "name" : "PlanOne",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1407153636262,
    "name" : "1407153636262",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1407153636261,
    "plan" : 1407153611768
  }, {
    "id" : 1407153791141,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1407153807487,
    "plan" : 1407153611768
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1407153636261,
    "name" : "DefaultState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153611768,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662909,
      "name" : "1587718662909",
      "comment" : "",
      "abstractPlan" : "PlanTwo.pml#1407153645238",
      "configuration" : null
    }, {
      "id" : 1587718662911,
      "name" : "1587718662911",
      "comment" : "",
      "abstractPlan" : "GoalPlan.pml#1402488870347",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153807487,
    "name" : "AttackState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153611768,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662914,
      "name" : "1587718662914",
      "comment" : "",
      "abstractPlan" : "PlanTwo.pml#1407153645238",
      "configuration" : null
    }, {
      "id" : 1587718662916,
      "name" : "1587718662916",
      "comment" : "",
      "abstractPlan" : "AttackPlan.pml#1402488634525",
      "configuration" : null
    }, {
      "id" : 1587718662918,
      "name" : "1587718662918",
      "comment" : "",
      "abstractPlan" : "Attack.pty#1407153314946",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}