{
  "id" : 1407153703092,
  "name" : "PlanFive",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1407153717809,
    "name" : "1407153717809",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1407153717808,
    "plan" : 1407153703092
  }, {
    "id" : 1407153972059,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1407153987910,
    "plan" : 1407153703092
  }, {
    "id" : 1407153973706,
    "name" : "DefendTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1402488486725",
    "state" : 1407153985762,
    "plan" : 1407153703092
  }, {
    "id" : 1407153975075,
    "name" : "MidFieldTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153536219",
    "state" : 1407153989550,
    "plan" : 1407153703092
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1407153717808,
    "name" : "DefaultState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153703092,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662616,
      "name" : "1587718662616",
      "comment" : "",
      "abstractPlan" : "GoalPlan.pml#1402488870347",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153985762,
    "name" : "DefendState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153703092,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662619,
      "name" : "1587718662619",
      "comment" : "",
      "abstractPlan" : "Defend.pml#1402488893641",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153987910,
    "name" : "AttackState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153703092,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662622,
      "name" : "1587718662622",
      "comment" : "",
      "abstractPlan" : "AttackPlan.pml#1402488634525",
      "configuration" : null
    }, {
      "id" : 1587718662624,
      "name" : "1587718662624",
      "comment" : "",
      "abstractPlan" : "Attack.pty#1407153314946",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153989550,
    "name" : "MidFieldState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153703092,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662627,
      "name" : "1587718662627",
      "comment" : "",
      "abstractPlan" : "MidFieldPlayPlan.pml#1402488770050",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}