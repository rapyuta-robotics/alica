{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 192405225773600915,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2851720313998235716,
      "positionWeb": {
        "x": 319,
        "y": 329
      },
      "state": 2231154692722562808,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2851720313998235716,
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
  "masterPlan": true,
  "name": "FourCornersTutorial",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "FourCorners.pml#1225570798912217901",
          "comment": "",
          "configuration": null,
          "id": 1515802129099037226,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3075001531216334549,
      "inTransitions": [
        2747861623143659037
      ],
      "name": "FourCorners",
      "outTransitions": [],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 734,
        "y": 325
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SpawnTurtle.beh#1689864767393644654",
          "comment": "",
          "configuration": null,
          "id": 1084340226690786551,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 192405225773600915,
      "id": 2231154692722562808,
      "inTransitions": [],
      "name": "SpawnTurtle",
      "outTransitions": [
        2747861623143659037
      ],
      "parentPlan": 2851720313998235716,
      "positionWeb": {
        "x": 471,
        "y": 316
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2747861623143659037,
      "inState": 2231154692722562808,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3075001531216334549,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2629062799832896160,
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
