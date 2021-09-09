{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1407153717809,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1407153717809",
      "plan": 1407153703092,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1407153717808,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1407153972059,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "AttackTask",
      "plan": 1407153703092,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1407153987910,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    },
    {
      "comment": "",
      "id": 1407153973706,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "DefendTask",
      "plan": 1407153703092,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 1407153985762,
      "successRequired": false,
      "task": "taskrepository.tsk#1402488486725"
    },
    {
      "comment": "",
      "id": 1407153975075,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MidFieldTask",
      "plan": 1407153703092,
      "positionWeb": {
        "x": 200,
        "y": 812
      },
      "state": 1407153989550,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153536219"
    }
  ],
  "frequency": 0,
  "id": 1407153703092,
  "masterPlan": false,
  "name": "PlanFive",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoalPlan.pml#1402488870347",
          "comment": "",
          "configuration": null,
          "id": 1587718662616,
          "name": "1587718662616"
        }
      ],
      "entryPoint": 1407153717809,
      "id": 1407153717808,
      "inTransitions": [],
      "name": "DefaultState",
      "outTransitions": [],
      "parentPlan": 1407153703092,
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
          "abstractPlan": "Defend.pml#1402488893641",
          "comment": "",
          "configuration": null,
          "id": 1587718662619,
          "name": "1587718662619"
        }
      ],
      "entryPoint": 1407153973706,
      "id": 1407153985762,
      "inTransitions": [],
      "name": "DefendState",
      "outTransitions": [],
      "parentPlan": 1407153703092,
      "positionWeb": {
        "x": 428,
        "y": 600
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "AttackPlan.pml#1402488634525",
          "comment": "",
          "configuration": null,
          "id": 1587718662622,
          "name": "1587718662622"
        },
        {
          "abstractPlan": "Attack.pty#1407153314946",
          "comment": "",
          "configuration": null,
          "id": 1587718662624,
          "name": "1587718662624"
        }
      ],
      "entryPoint": 1407153972059,
      "id": 1407153987910,
      "inTransitions": [],
      "name": "AttackState",
      "outTransitions": [],
      "parentPlan": 1407153703092,
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
          "abstractPlan": "MidFieldPlayPlan.pml#1402488770050",
          "comment": "",
          "configuration": null,
          "id": 1587718662627,
          "name": "1587718662627"
        }
      ],
      "entryPoint": 1407153975075,
      "id": 1407153989550,
      "inTransitions": [],
      "name": "MidFieldState",
      "outTransitions": [],
      "parentPlan": 1407153703092,
      "positionWeb": {
        "x": 428,
        "y": 800
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