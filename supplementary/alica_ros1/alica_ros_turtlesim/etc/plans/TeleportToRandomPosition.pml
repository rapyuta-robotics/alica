{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 1417221173347283069,
      "key": "xmin",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 134046690438663400,
      "key": "xmax",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 563252773862417130,
      "key": "ymin",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2382626652775529582,
      "key": "ymax",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 598868303244083964,
      "key": "x",
      "type": "double"
    },
    {
      "access": "protected",
      "comment": "",
      "id": 2712475924298828232,
      "key": "y",
      "type": "double"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 716543537979003414,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 847199748749956244,
      "positionWeb": {
        "x": 271.5532139093783,
        "y": 303.9905163329821
      },
      "state": 815756385850055878,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 847199748749956244,
  "inheritBlackboard": false,
  "libraryName": "libalica-turtlesim",
  "masterPlan": false,
  "name": "TeleportToRandomPosition",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "GenerateRandom.beh#3356053372497284615",
          "comment": "",
          "configuration": null,
          "id": 172292232628300347,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": "xmin",
                "value": null
              },
              {
                "childKey": "max",
                "parentKey": "xmax",
                "value": null
              }
            ],
            "output": [
              {
                "childKey": "value",
                "parentKey": "x"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": 716543537979003414,
      "id": 815756385850055878,
      "inTransitions": [],
      "name": "GenerateRandomX",
      "outTransitions": [
        1488022082942751415
      ],
      "parentPlan": 847199748749956244,
      "positionWeb": {
        "x": 389.78292939936773,
        "y": 288.7512283235457
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Teleport.beh#4085572422059465423",
          "comment": "",
          "configuration": null,
          "id": 2086117606402602846,
          "keyMapping": {
            "input": [
              {
                "childKey": "x",
                "parentKey": "x",
                "value": null
              },
              {
                "childKey": "y",
                "parentKey": "y",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1435429019417319391,
      "inTransitions": [
        705459995715723320
      ],
      "name": "Teleport",
      "outTransitions": [
        3964450176861985360
      ],
      "parentPlan": 847199748749956244,
      "positionWeb": {
        "x": 1040.6616392144417,
        "y": 284.21088432080336
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3715359869158732014,
      "inTransitions": [
        3964450176861985360
      ],
      "name": "Success",
      "outTransitions": [],
      "parentPlan": 847199748749956244,
      "positionWeb": {
        "x": 1325.1738610144441,
        "y": 281.3943460822985
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
          "abstractPlan": "GenerateRandom.beh#3356053372497284615",
          "comment": "",
          "configuration": null,
          "id": 2546391469599184355,
          "keyMapping": {
            "input": [
              {
                "childKey": "min",
                "parentKey": "ymin",
                "value": null
              },
              {
                "childKey": "max",
                "parentKey": "ymax",
                "value": null
              }
            ],
            "output": [
              {
                "childKey": "value",
                "parentKey": "y"
              }
            ]
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 32950839872639430,
      "inTransitions": [
        1488022082942751415
      ],
      "name": "GenerateRandomY",
      "outTransitions": [
        705459995715723320
      ],
      "parentPlan": 847199748749956244,
      "positionWeb": {
        "x": 710.2464886248355,
        "y": 289.88551772476035
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
      "id": 3964450176861985360,
      "inState": 1435429019417319391,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3715359869158732014,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1374213844849286464,
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
      "id": 1488022082942751415,
      "inState": 815756385850055878,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 32950839872639430,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4091298319182661256,
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
      "id": 705459995715723320,
      "inState": 32950839872639430,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1435429019417319391,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2597932705904404824,
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
