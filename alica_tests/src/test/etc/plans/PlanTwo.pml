{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1407153656782,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1407153656782",
      "plan": 1407153645238,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1407153656781,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1407153821287,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "AttackTask",
      "plan": 1407153645238,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1407153860891,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    },
    {
      "comment": "",
      "id": 1407153842648,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "DefendTask",
      "plan": 1407153645238,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 1407153869754,
      "successRequired": false,
      "task": "taskrepository.tsk#1402488486725"
    }
  ],
  "frequency": 0,
  "id": 1407153645238,
  "masterPlan": false,
  "name": "PlanTwo",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanThree.pml#1407153663917",
          "comment": "",
          "configuration": null,
          "id": 1587718662791,
          "name": "1587718662791"
        },
        {
          "abstractPlan": "GoalPlan.pml#1402488870347",
          "comment": "",
          "configuration": null,
          "id": 1587718662793,
          "name": "1587718662793"
        }
      ],
      "entryPoint": 1407153656782,
      "id": 1407153656781,
      "inTransitions": [],
      "name": "DefaultState",
      "outTransitions": [],
      "parentPlan": 1407153645238,
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
          "abstractPlan": "PlanThree.pml#1407153663917",
          "comment": "",
          "configuration": null,
          "id": 1587718662796,
          "name": "1587718662796"
        },
        {
          "abstractPlan": "AttackPlan.pml#1402488634525",
          "comment": "",
          "configuration": null,
          "id": 1587718662798,
          "name": "1587718662798"
        },
        {
          "abstractPlan": "Attack.pty#1407153314946",
          "comment": "",
          "configuration": null,
          "id": 1587718662800,
          "name": "1587718662800"
        }
      ],
      "entryPoint": 1407153821287,
      "id": 1407153860891,
      "inTransitions": [],
      "name": "AttackState",
      "outTransitions": [],
      "parentPlan": 1407153645238,
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
          "abstractPlan": "PlanThree.pml#1407153663917",
          "comment": "",
          "configuration": null,
          "id": 1587718662803,
          "name": "1587718662803"
        },
        {
          "abstractPlan": "Defend.pml#1402488893641",
          "comment": "",
          "configuration": null,
          "id": 1587718662805,
          "name": "1587718662805"
        }
      ],
      "entryPoint": 1407153842648,
      "id": 1407153869754,
      "inTransitions": [],
      "name": "DefendState",
      "outTransitions": [],
      "parentPlan": 1407153645238,
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