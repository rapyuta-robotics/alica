{
  "blackboard": [
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
      "id": 3993490433617687004,
      "key": "pick_y",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 1282199217470743347,
      "key": "drop_x",
      "type": "double"
    },
    {
      "access": "input",
      "comment": "task metadata",
      "id": 3936152552717061281,
      "key": "task",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "The list of keys to be parsed from the metadata & put onto this plan's blackboard",
      "id": 2793016454011826362,
      "key": "blackboardKeys",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 3117539008308143553,
      "key": "pick_x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "Pointer to this plan's blackboard",
      "id": 115586003075733571,
      "key": "blackboard",
      "type": "std::any"
    },
    {
      "access": "protected",
      "comment": "Pointer to this plan's blackboard blueprint",
      "id": 2069627758244916054,
      "key": "blackboardBlueprint",
      "type": "std::any"
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
        "x": 349.46191346739795,
        "y": 392.61283770058907
      },
      "state": 2960013304925378014,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 674771508287965487,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "libalica-turtlesim",
  "masterPlan": false,
  "name": "TransportWorkflow",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
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
        "x": 2068.529322093354,
        "y": 392.5182966606626
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
        "x": 1785.7797956146997,
        "y": 393.6820538434737
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
        "x": 1481.2715863761975,
        "y": 393.71396775203294
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
        "x": 1192.5585609153657,
        "y": 396.237367487433
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
      "entryPoint": null,
      "id": 2696583768627107612,
      "inTransitions": [
        4226873529897699302
      ],
      "name": "GoToPickLocation",
      "outTransitions": [
        1247410926919398841
      ],
      "parentPlan": 674771508287965487,
      "positionWeb": {
        "x": 905.3843561542031,
        "y": 388.0767830426448
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PopulateBlackboardFromJson.beh#3218372711073374685",
          "comment": "",
          "configuration": null,
          "id": 1543811457707879254,
          "keyMapping": {
            "input": [
              {
                "childKey": "blackboard",
                "parentKey": "blackboard",
                "value": null
              },
              {
                "childKey": "blackboardBlueprint",
                "parentKey": "blackboardBlueprint",
                "value": null
              },
              {
                "childKey": "blackboardKeys",
                "parentKey": "blackboardKeys",
                "value": null
              },
              {
                "childKey": "json",
                "parentKey": "task",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1953638029732664149,
      "id": 2960013304925378014,
      "inTransitions": [],
      "name": "PopulateBlackboard",
      "outTransitions": [
        4226873529897699302
      ],
      "parentPlan": 674771508287965487,
      "positionWeb": {
        "x": 545.0320942514726,
        "y": 383.97403837716587
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
      "id": 700752702149744653,
      "inState": 4095445754645596712,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3507842964888136923,
      "pointsWeb": [],
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
      "id": 315024965563498857,
      "inState": 3327829358050489991,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1892335939577056309,
      "pointsWeb": [],
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
      "id": 4226873529897699302,
      "inState": 2960013304925378014,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2696583768627107612,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2635615433685131100,
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
      "pointsWeb": [],
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
      "pointsWeb": [],
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
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
