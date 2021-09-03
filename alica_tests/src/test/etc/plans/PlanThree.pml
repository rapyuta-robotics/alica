{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1407153675525,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1407153675525",
      "plan": 1407153663917,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1407153675524,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    },
    {
      "comment": "",
      "id": 1407153896585,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MidFieldTask",
      "plan": 1407153663917,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1407153914126,
      "successRequired": false,
      "task": "taskrepository.tsk#1407153536219"
    },
    {
      "comment": "",
      "id": 1407153899241,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "DefendTask",
      "plan": 1407153663917,
      "positionWeb": {
        "x": 200,
        "y": 612
      },
      "state": 1407153916646,
      "successRequired": false,
      "task": "taskrepository.tsk#1402488486725"
    }
  ],
  "frequency": 0,
  "id": 1407153663917,
  "masterPlan": false,
  "name": "PlanThree",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanFour.pml#1407153683051",
          "comment": "",
          "configuration": null,
          "id": 1587718662883,
          "name": "1587718662883"
        },
        {
          "abstractPlan": "GoalPlan.pml#1402488870347",
          "comment": "",
          "configuration": null,
          "id": 1587718662885,
          "name": "1587718662885"
        }
      ],
      "entryPoint": 1407153675525,
      "id": 1407153675524,
      "inTransitions": [],
      "name": "DefaultState",
      "outTransitions": [],
      "parentPlan": 1407153663917,
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
          "abstractPlan": "PlanFour.pml#1407153683051",
          "comment": "",
          "configuration": null,
          "id": 1587718662888,
          "name": "1587718662888"
        },
        {
          "abstractPlan": "MidFieldPlayPlan.pml#1402488770050",
          "comment": "",
          "configuration": null,
          "id": 1587718662890,
          "name": "1587718662890"
        }
      ],
      "entryPoint": 1407153896585,
      "id": 1407153914126,
      "inTransitions": [],
      "name": "MidFieldState",
      "outTransitions": [],
      "parentPlan": 1407153663917,
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
          "abstractPlan": "PlanFour.pml#1407153683051",
          "comment": "",
          "configuration": null,
          "id": 1587718662893,
          "name": "1587718662893"
        },
        {
          "abstractPlan": "Defend.pml#1402488893641",
          "comment": "",
          "configuration": null,
          "id": 1587718662895,
          "name": "1587718662895"
        }
      ],
      "entryPoint": 1407153899241,
      "id": 1407153916646,
      "inTransitions": [],
      "name": "DefendState",
      "outTransitions": [],
      "parentPlan": 1407153663917,
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