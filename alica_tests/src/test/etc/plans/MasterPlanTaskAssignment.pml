{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1407152758499,
      "maxCardinality": 5,
      "minCardinality": 2,
      "name": "1407152758499",
      "plan": 1407152758497,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1407152758498,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1407152894887,
      "maxCardinality": 1000,
      "minCardinality": 0,
      "name": "1407152894887",
      "plan": 1407152758497,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1407152951886,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    },
    {
      "comment": "",
      "id": 1407152900425,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1407152900425",
      "plan": 1407152758497,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 1407152969078,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153536219"
    },
    {
      "comment": "",
      "id": 1407152902493,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "DefendTask",
      "plan": 1407152758497,
      "positionWeb": {
        "x": 200,
        "y": 812
      },
      "state": 1407152962295,
      "successRequired": false,
      "task": "taskrepository.tsk#1402488486725"
    }
  ],
  "frequency": 0,
  "id": 1407152758497,
  "masterPlan": true,
  "name": "MasterPlanTaskAssignment",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanOne.pml#1407153611768",
          "comment": "",
          "configuration": null,
          "id": 1587718662976,
          "name": "1587718662976"
        },
        {
          "abstractPlan": "Attack.pty#1407153314946",
          "comment": "",
          "configuration": null,
          "id": 1587718662978,
          "name": "1587718662978"
        }
      ],
      "entryPoint": 1407152758499,
      "id": 1407152758498,
      "inTransitions": [],
      "name": "AttackFirst",
      "outTransitions": [],
      "parentPlan": 1407152758497,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "MidFieldPlayPlan.pml#1402488770050",
          "comment": "",
          "configuration": null,
          "id": 1587718662981,
          "name": "1587718662981"
        },
        {
          "abstractPlan": "PlanOne.pml#1407153611768",
          "comment": "",
          "configuration": null,
          "id": 1587718662983,
          "name": "1587718662983"
        }
      ],
      "entryPoint": 1407152894887,
      "id": 1407152951886,
      "inTransitions": [],
      "name": "MidField",
      "outTransitions": [],
      "parentPlan": 1407152758497,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Defend.pml#1402488893641",
          "comment": "",
          "configuration": null,
          "id": 1587718662986,
          "name": "1587718662986"
        },
        {
          "abstractPlan": "PlanOne.pml#1407153611768",
          "comment": "",
          "configuration": null,
          "id": 1587718662988,
          "name": "1587718662988"
        }
      ],
      "entryPoint": 1407152902493,
      "id": 1407152962295,
      "inTransitions": [],
      "name": "Defend",
      "outTransitions": [],
      "parentPlan": 1407152758497,
      "positionWeb": {
        "x": 428,
        "y": 800
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoalPlan.pml#1402488870347",
          "comment": "",
          "configuration": null,
          "id": 1587718662991,
          "name": "1587718662991"
        },
        {
          "abstractPlan": "PlanOne.pml#1407153611768",
          "comment": "",
          "configuration": null,
          "id": 1587718662993,
          "name": "1587718662993"
        }
      ],
      "entryPoint": 1407152900425,
      "id": 1407152969078,
      "inTransitions": [],
      "name": "Goal",
      "outTransitions": [],
      "parentPlan": 1407152758497,
      "positionWeb": {
        "x": 428,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.1,
  "variables": []
}