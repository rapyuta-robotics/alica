{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 787860857360251517,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 3688442640782714083,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1286362603130390415,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 3688442640782714083,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "BehaviourRunSchedulingTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "BehAAA.beh#1629895901559",
          "comment": "",
          "configuration": null,
          "id": 4213282615326719804,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 787860857360251517,
      "id": 1286362603130390415,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        4610641909683513648
      ],
      "parentPlan": 3688442640782714083,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2220539979049614276,
      "inTransitions": [
        4610641909683513648
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 3688442640782714083,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": null,
      "id": 4610641909683513648,
      "inState": 1286362603130390415,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2220539979049614276,
      "pointsWeb": [
        {
          "x": 586,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 74329762344731703,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
