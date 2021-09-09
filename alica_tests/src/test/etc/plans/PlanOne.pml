{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1407153636262,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1407153636262",
      "plan": 1407153611768,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1407153636261,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1407153791141,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "AttackTask",
      "plan": 1407153611768,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1407153807487,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153522080"
    }
  ],
  "frequency": 0,
  "id": 1407153611768,
  "masterPlan": false,
  "name": "PlanOne",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanTwo.pml#1407153645238",
          "comment": "",
          "configuration": null,
          "id": 1587718662909,
          "name": "1587718662909"
        },
        {
          "abstractPlan": "GoalPlan.pml#1402488870347",
          "comment": "",
          "configuration": null,
          "id": 1587718662911,
          "name": "1587718662911"
        }
      ],
      "entryPoint": 1407153636262,
      "id": 1407153636261,
      "inTransitions": [],
      "name": "DefaultState",
      "outTransitions": [],
      "parentPlan": 1407153611768,
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
          "abstractPlan": "PlanTwo.pml#1407153645238",
          "comment": "",
          "configuration": null,
          "id": 1587718662914,
          "name": "1587718662914"
        },
        {
          "abstractPlan": "AttackPlan.pml#1402488634525",
          "comment": "",
          "configuration": null,
          "id": 1587718662916,
          "name": "1587718662916"
        },
        {
          "abstractPlan": "Attack.pty#1407153314946",
          "comment": "",
          "configuration": null,
          "id": 1587718662918,
          "name": "1587718662918"
        }
      ],
      "entryPoint": 1407153791141,
      "id": 1407153807487,
      "inTransitions": [],
      "name": "AttackState",
      "outTransitions": [],
      "parentPlan": 1407153611768,
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