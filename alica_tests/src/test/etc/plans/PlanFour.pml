{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1407153696703,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1407153696703",
      "plan": 1407153683051,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1407153696702,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1407153949327,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "AttackTask",
      "plan": 1407153683051,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1407153959299,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    }
  ],
  "frequency": 0,
  "id": 1407153683051,
  "masterPlan": false,
  "name": "PlanFour",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanFive.pml#1407153703092",
          "comment": "",
          "configuration": null,
          "id": 1587718663048,
          "name": "1587718663048"
        },
        {
          "abstractPlan": "GoalPlan.pml#1402488870347",
          "comment": "",
          "configuration": null,
          "id": 1587718663050,
          "name": "1587718663050"
        }
      ],
      "entryPoint": 1407153696703,
      "id": 1407153696702,
      "inTransitions": [],
      "name": "DefaultState",
      "outTransitions": [],
      "parentPlan": 1407153683051,
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
          "abstractPlan": "PlanFive.pml#1407153703092",
          "comment": "",
          "configuration": null,
          "id": 1587718663054,
          "name": "1587718663054"
        },
        {
          "abstractPlan": "AttackPlan.pml#1402488634525",
          "comment": "",
          "configuration": null,
          "id": 1587718663056,
          "name": "1587718663056"
        },
        {
          "abstractPlan": "Attack.pty#1407153314946",
          "comment": "",
          "configuration": null,
          "id": 1587718663058,
          "name": "1587718663058"
        }
      ],
      "entryPoint": 1407153949327,
      "id": 1407153959299,
      "inTransitions": [],
      "name": "AttackState",
      "outTransitions": [],
      "parentPlan": 1407153683051,
      "positionWeb": {
        "x": 428,
        "y": 400
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