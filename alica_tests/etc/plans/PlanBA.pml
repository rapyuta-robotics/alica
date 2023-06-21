{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1629896091322,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1629896091322",
      "plan": 1629895873188,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1629896094706,
      "successRequired": false,
      "task": "taskrepository.tsk#1613371619454"
    }
  ],
  "frequency": 60,
  "id": 1629895873188,
  "implementationName": "PlanA",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "PlanBA",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "BehBAA.beh#1629895911592",
          "comment": "",
          "configuration": null,
          "id": 1629896123291,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1629896123291"
        }
      ],
      "entryPoint": 1629896091322,
      "id": 1629896094706,
      "inTransitions": [],
      "name": "BehBAA",
      "outTransitions": [
        1433587281699428693
      ],
      "parentPlan": 1629895873188,
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
      "id": 2164308778216412252,
      "inTransitions": [
        1433587281699428693
      ],
      "name": "",
      "outTransitions": [],
      "parentPlan": 1629895873188,
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
      "condition": "ConditionRepository.cnd#2",
      "id": 1433587281699428693,
      "inState": 1629896094706,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "SuccessState",
      "outState": 2164308778216412252,
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
        "id": 3672994099534071266,
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
