{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 1282199217470743347,
      "key": "drop_x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2777554424664838944,
      "key": "drop_y",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3117539008308143553,
      "key": "pick_x",
      "type": "double"
    },
    {
      "access": "input",
      "comment": "",
      "id": 3936152552717061281,
      "key": "data",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3993490433617687004,
      "key": "pick_y",
      "type": "double"
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
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "TransportWorkflow",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 1828431111047595794,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": "drop_x",
                "value": null
              },
              {
                "childKey": "goal_y",
                "parentKey": "drop_y",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1892335939577056309,
      "inTransitions": [
        315024965563498857
      ],
      "name": "GoToDropLocation",
      "outTransitions": [
        1984885427694029024
      ],
      "parentPlan": 674771508287965487,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GoTo.beh#2797939494274869075",
          "comment": "",
          "configuration": null,
          "id": 47098071405004600,
          "keyMapping": {
            "input": [
              {
                "childKey": "goal_x",
                "parentKey": "pick_x",
                "value": null
              },
              {
                "childKey": "goal_y",
                "parentKey": "pick_y",
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
      "name": "GoToPickLocation",
      "outTransitions": [
        1247410926919398841
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
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 2840947820777305843,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": null,
                "value": "pick_drop_done"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3327829358050489991,
      "inTransitions": [
        1247410926919398841
      ],
      "name": "Pick",
      "outTransitions": [
        315024965563498857
      ],
      "parentPlan": 674771508287965487,
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
      "id": 3507842964888136923,
      "inTransitions": [
        700752702149744653
      ],
      "name": "Transported",
      "outTransitions": [],
      "parentPlan": 674771508287965487,
      "positionWeb": {
        "x": 1460,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 435140749561177907,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": null,
                "value": "pick_drop_done"
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 4095445754645596712,
      "inTransitions": [
        1984885427694029024
      ],
      "name": "Drop",
      "outTransitions": [
        700752702149744653
      ],
      "parentPlan": 674771508287965487,
      "positionWeb": {
        "x": 1202,
        "y": 200
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
      "id": 315024965563498857,
      "inState": 3327829358050489991,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1892335939577056309,
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
        "id": 2549758766918075490,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 700752702149744653,
      "inState": 4095445754645596712,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3507842964888136923,
      "pointsWeb": [
        {
          "x": 1360,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4313604777492288450,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 1247410926919398841,
      "inState": 2696583768627107612,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3327829358050489991,
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
        "id": 1371847483626799114,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 1984885427694029024,
      "inState": 1892335939577056309,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4095445754645596712,
      "pointsWeb": [
        {
          "x": 1102,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 786873304476758007,
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
