{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1282199217470743347,
      "key": "target_x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 2777554424664838944,
      "key": "target_y",
      "type": "double"
    },
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 3936152552717061281,
      "key": "data",
      "type": "std::string"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1953638029732664149,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 674771508287965487,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2696583768627107612,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 674771508287965487,
  "implementationName": "PopulateBlackboard",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "TeleportToWorkflow",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Teleport.beh#4085572422059465423",
          "comment": "",
          "configuration": null,
          "id": 3447500310080826891,
          "keyMapping": {
            "input": [
              {
                "childKey": "y",
                "parentKey": "target_y",
                "value": null
              },
              {
                "childKey": "x",
                "parentKey": "target_x",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1953638029732664149,
      "id": 2696583768627107612,
      "inTransitions": [],
      "name": "TeleportToLocation",
      "outTransitions": [
        1598806969835857442
      ],
      "parentPlan": 674771508287965487,
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
      "id": 3507842964888136923,
      "inTransitions": [
        1598806969835857442
      ],
      "name": "Teleported",
      "outTransitions": [],
      "parentPlan": 674771508287965487,
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
      "condition": "ConditionRepository.cnd#1",
      "id": 1598806969835857442,
      "inState": 2696583768627107612,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3507842964888136923,
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
        "id": 266662766236354683,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
