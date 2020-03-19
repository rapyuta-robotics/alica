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
    "abstractPlans" : [ "PlanTwo.pml#1407153645238", "GoalPlan.pml#1402488870347" ],
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
    "abstractPlans" : [ "PlanTwo.pml#1407153645238", "AttackPlan.pml#1402488634525", "Attack.pty#1407153314946" ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}