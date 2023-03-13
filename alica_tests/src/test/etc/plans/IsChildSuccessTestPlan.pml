{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2614086846208556728,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2027765331130289429,
      "positionWeb": {
        "x": 195,
        "y": 285
      },
      "state": 4328347657318112535,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2027765331130289429,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "IsChildSuccessTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SuccessOnInitBeh.beh#3821787310391665935",
          "comment": "",
          "configuration": null,
          "id": 655124321119057103,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2614086846208556728,
      "id": 4328347657318112535,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        2590151640025681691
      ],
      "parentPlan": 2027765331130289429,
      "positionWeb": {
        "x": 573,
        "y": 306
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4068992413297046586,
      "inTransitions": [
        2590151640025681691
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 2027765331130289429,
      "positionWeb": {
        "x": 928,
        "y": 340
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
      "condition": "ConditionRepository.cnd#3290850633957473696",
      "id": 2590151640025681691,
      "inState": 4328347657318112535,
      "keyMapping": {
        "input": [
          {
            "childKey": "childName",
            "parentKey": null,
            "value": "SuccessOnInitBeh"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4068992413297046586,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1382846002212836995,
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
