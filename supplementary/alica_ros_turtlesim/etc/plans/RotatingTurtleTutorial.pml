{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2287376619708516790,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3047149772109767003,
      "positionWeb": {
        "x": 235,
        "y": 308
      },
      "state": 34977861995250952,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 3047149772109767003,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": true,
  "name": "RotatingTurtleTutorial",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SpawnTurtle.beh#1689864767393644654",
          "comment": "",
          "configuration": null,
          "id": 1244979919906966056,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        },
        {
          "abstractPlan": "RotateTurtle.beh#2948815528667048378",
          "comment": "",
          "configuration": null,
          "id": 2379999259387878547,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2287376619708516790,
      "id": 34977861995250952,
      "inTransitions": [],
      "name": "State",
      "outTransitions": [],
      "parentPlan": 3047149772109767003,
      "positionWeb": {
        "x": 434,
        "y": 298
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.0,
  "variables": []
}
