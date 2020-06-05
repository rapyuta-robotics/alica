{
  "id" : 1407152758497,
  "name" : "MasterPlanTaskAssignment",
  "comment" : "",
  "relativeDirectory" : "",
  "variables" : [ ],
  "masterPlan" : true,
  "utilityThreshold" : 0.1,
  "preCondition" : null,
  "runtimeCondition" : null,
  "entryPoints" : [ {
    "id" : 1407152758499,
    "name" : "1407152758499",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 2,
    "maxCardinality" : 5,
    "task" : "taskrepository.tsk#1225112227903",
    "state" : 1407152758498,
    "plan" : 1407152758497
  }, {
    "id" : 1407152894887,
    "name" : "1407152894887",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 1000,
    "task" : "taskrepository.tsk#1407153522080",
    "state" : 1407152951886,
    "plan" : 1407152758497
  }, {
    "id" : 1407152900425,
    "name" : "1407152900425",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1407153536219",
    "state" : 1407152969078,
    "plan" : 1407152758497
  }, {
    "id" : 1407152902493,
    "name" : "DefendTask",
    "comment" : "",
    "successRequired" : false,
    "minCardinality" : 0,
    "maxCardinality" : 2147483647,
    "task" : "taskrepository.tsk#1402488486725",
    "state" : 1407152962295,
    "plan" : 1407152758497
  } ],
  "states" : [ {
    "type" : "State",
    "id" : 1407152758498,
    "name" : "AttackFirst",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662976,
      "name" : "1587718662976",
      "comment" : "",
      "abstractPlan" : "PlanOne.pml#1407153611768",
      "configuration" : null
    }, {
      "id" : 1587718662978,
      "name" : "1587718662978",
      "comment" : "",
      "abstractPlan" : "Attack.pty#1407153314946",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407152951886,
    "name" : "MidField",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662981,
      "name" : "1587718662981",
      "comment" : "",
      "abstractPlan" : "MidFieldPlayPlan.pml#1402488770050",
      "configuration" : null
    }, {
      "id" : 1587718662983,
      "name" : "1587718662983",
      "comment" : "",
      "abstractPlan" : "PlanOne.pml#1407153611768",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407152962295,
    "name" : "Defend",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662986,
      "name" : "1587718662986",
      "comment" : "",
      "abstractPlan" : "Defend.pml#1402488893641",
      "configuration" : null
    }, {
      "id" : 1587718662988,
      "name" : "1587718662988",
      "comment" : "",
      "abstractPlan" : "PlanOne.pml#1407153611768",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  }, {
    "type" : "State",
    "id" : 1407152969078,
    "name" : "Goal",
    "comment" : "",
    "entryPoint" : null,
    "parentPlan" : 1407152758497,
    "confAbstractPlanWrappers" : [ {
      "id" : 1587718662991,
      "name" : "1587718662991",
      "comment" : "",
      "abstractPlan" : "GoalPlan.pml#1402488870347",
      "configuration" : null
    }, {
      "id" : 1587718662993,
      "name" : "1587718662993",
      "comment" : "",
      "abstractPlan" : "PlanOne.pml#1407153611768",
      "configuration" : null
    } ],
    "variableBindings" : [ ],
    "outTransitions" : [ ],
    "inTransitions" : [ ]
  } ],
  "transitions" : [ ],
  "synchronisations" : [ ]
}