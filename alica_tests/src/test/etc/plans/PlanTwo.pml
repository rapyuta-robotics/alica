{
  "id" : 1407153645238,
  "name" : "PlanTwo",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : false,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1407153656782,
    "name" : "1407153656782",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1407153656781,
    "plan" : 1407153645238
  }, {
    "id" : 1407153821287,
    "name" : "AttackTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1407153860891,
    "plan" : 1407153645238
  }, {
    "id" : 1407153842648,
    "name" : "DefendTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1402488486725",
    "state" : 1407153869754,
    "plan" : 1407153645238
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1407153656781,
    "name" : "DefaultState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153645238,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662791,
      "name" : "1587718662791",
      "comment" : "",
      "abstractPlan" : "PlanThree.pml#1407153663917",
      "configuration" : null
    }, {
      "id" : 1587718662793,
      "name" : "1587718662793",
      "comment" : "",
      "abstractPlan" : "GoalPlan.pml#1402488870347",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153860891,
    "name" : "AttackState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153645238,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662796,
      "name" : "1587718662796",
      "comment" : "",
      "abstractPlan" : "PlanThree.pml#1407153663917",
      "configuration" : null
    }, {
      "id" : 1587718662798,
      "name" : "1587718662798",
      "comment" : "",
      "abstractPlan" : "AttackPlan.pml#1402488634525",
      "configuration" : null
    }, {
      "id" : 1587718662800,
      "name" : "1587718662800",
      "comment" : "",
      "abstractPlan" : "Attack.pty#1407153314946",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407153869754,
    "name" : "DefendState",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407153645238,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662803,
      "name" : "1587718662803",
      "comment" : "",
      "abstractPlan" : "PlanThree.pml#1407153663917",
      "configuration" : null
    }, {
      "id" : 1587718662805,
      "name" : "1587718662805",
      "comment" : "",
      "abstractPlan" : "Defend.pml#1402488893641",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}