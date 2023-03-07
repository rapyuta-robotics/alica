{
  "blackboard": [
    {
      "access": "protected",
      "comment": "This is a blackboard entry for testing",
      "id": 3046890668195475143,
      "key": "masterKey",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4058387577648167302,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "ParameterPassingMasterEP",
      "plan": 1179066429431332055,
      "positionWeb": {
        "x": 265,
        "y": 263
      },
      "state": 2069338196796962570,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1179066429431332055,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "TestParameterPassingPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestParameterPassingSubPlan.pml#1692837668719979457",
          "comment": "",
          "configuration": null,
          "id": 105160539449888459,
          "keyMapping": {
            "input": [
              {
                "childKey": "planInputFromMaster",
                "parentKey": "masterKey",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 4058387577648167302,
      "id": 2069338196796962570,
      "inTransitions": [],
      "name": "ParameterPassingRunSubPlan",
      "outTransitions": [
        3160687683964723810
      ],
      "parentPlan": 1179066429431332055,
      "positionWeb": {
        "x": 627,
        "y": 277
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1476982130239901494,
      "inTransitions": [
        3160687683964723810
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1179066429431332055,
      "positionWeb": {
        "x": 953.1820814141756,
        "y": 301.85974008796035
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
      "condition": "ConditionRepository.cnd#1",
      "id": 3160687683964723810,
      "inState": 2069338196796962570,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1476982130239901494,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1378977514582254718,
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
