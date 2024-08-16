{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1373883389746082897,
      "key": "workflow",
      "type": "std::string"
    },
    {
      "access": "input",
      "comment": "",
      "defaultValue": null,
      "id": 4268930889364842881,
      "key": "data",
      "type": "std::string"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3143738899596840342,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1923957580593038667,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1572902355338598541,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 1923957580593038667,
  "implementationName": "PopulateBlackboard",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "CustomWorkflow",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3143738899596840342,
      "id": 1572902355338598541,
      "inTransitions": [],
      "name": "Decider",
      "outTransitions": [
        1418800620508557672,
        3440050605748373299
      ],
      "parentPlan": 1923957580593038667,
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
          "abstractPlan": "MoveToWorkflow.pml#3683805017237692753",
          "comment": "",
          "configuration": null,
          "id": 4062715461950206158,
          "keyMapping": {
            "input": [
              {
                "childKey": "data",
                "parentKey": "data",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2165676525192089877,
      "inTransitions": [
        3440050605748373299
      ],
      "name": "AdhocMoveTask",
      "outTransitions": [
        3330333657712061897
      ],
      "parentPlan": 1923957580593038667,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TeleportToWorkflow.pml#674771508287965487",
          "comment": "",
          "configuration": null,
          "id": 2272668359491616673,
          "keyMapping": {
            "input": [
              {
                "childKey": "data",
                "parentKey": "data",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2632266932596278781,
      "inTransitions": [
        1418800620508557672
      ],
      "name": "TransportTask",
      "outTransitions": [
        2651058682523806223
      ],
      "parentPlan": 1923957580593038667,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4448376615665531967,
      "inTransitions": [
        2651058682523806223,
        3330333657712061897
      ],
      "name": "Completed",
      "outTransitions": [],
      "parentPlan": 1923957580593038667,
      "positionWeb": {
        "x": 944,
        "y": 400
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
      "condition": "ConditionRepository.cnd#976473172990372064",
      "id": 1418800620508557672,
      "inState": 1572902355338598541,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": "workflow",
            "value": null
          },
          {
            "childKey": "right",
            "parentKey": null,
            "value": "teleportTo"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2632266932596278781,
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
        "id": 2752986009859690318,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2651058682523806223,
      "inState": 2632266932596278781,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4448376615665531967,
      "pointsWeb": [
        {
          "x": 844,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 872737776606686249,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 3330333657712061897,
      "inState": 2165676525192089877,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4448376615665531967,
      "pointsWeb": [
        {
          "x": 844,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1837767846344475889,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#976473172990372064",
      "id": 3440050605748373299,
      "inState": 1572902355338598541,
      "keyMapping": {
        "input": [
          {
            "childKey": "left",
            "parentKey": "workflow",
            "value": null
          },
          {
            "childKey": "right",
            "parentKey": null,
            "value": "moveTo"
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2165676525192089877,
      "pointsWeb": [
        {
          "x": 586,
          "y": 429
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4121802518090307623,
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
