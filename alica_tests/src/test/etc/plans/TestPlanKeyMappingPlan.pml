{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 2800258364181388921,
      "key": "parentPlanKey",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2412782515831730712,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 4104108554421232939,
      "positionWeb": {
        "x": 265,
        "y": 269
      },
      "state": 315760183192378005,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 4104108554421232939,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "TestPlanKeyMappingPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestPlanKeyMappingSubPlan.pml#1282671484567034780",
          "comment": "",
          "configuration": null,
          "id": 1872901335542231817,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2412782515831730712,
      "id": 315760183192378005,
      "inTransitions": [],
      "name": "RunTestState",
      "outTransitions": [
        959778036987945685
      ],
      "parentPlan": 4104108554421232939,
      "positionWeb": {
        "x": 678,
        "y": 279
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 844143289784397586,
      "inTransitions": [
        959778036987945685
      ],
      "name": "",
      "outTransitions": [],
      "parentPlan": 4104108554421232939,
      "positionWeb": {
        "x": 963.709141244464,
        "y": 289.6032822789
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
      "id": 959778036987945685,
      "inState": 315760183192378005,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 844143289784397586,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1149746447710032253,
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
