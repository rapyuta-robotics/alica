{
  "id" : 1407153683051,
  "name" : "PlanFour",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1407153696703,
    "name" : "1407153696703",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1407153696702,
    "plan" : 1407153683051
  }, {
    "id" : 1407153949327,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1407153959299,
    "plan" : 1407153683051
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1407153696702,
    "name" : "DefaultState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153683051,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718663048,
      "name" : "1587718663048",
      "comment" : "",
      "abstractPlan" : "PlanFive.pml#1407153703092",
      "configuration" : null
    }, {
      "id" : 1587718663050,
      "name" : "1587718663050",
      "comment" : "",
      "abstractPlan" : "GoalPlan.pml#1402488870347",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153959299,
    "name" : "AttackState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153683051,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718663054,
      "name" : "1587718663054",
      "comment" : "",
      "abstractPlan" : "PlanFive.pml#1407153703092",
      "configuration" : null
    }, {
      "id" : 1587718663056,
      "name" : "1587718663056",
      "comment" : "",
      "abstractPlan" : "AttackPlan.pml#1402488634525",
      "configuration" : null
    }, {
      "id" : 1587718663058,
      "name" : "1587718663058",
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