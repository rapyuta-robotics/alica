{
  "blackboard": [
    {
      "access": "protected",
      "comment": "Subscriber topic to listen for trigger msgs to join the formation",
      "id": 4387163444028990842,
      "key": "join_formation_topic",
      "type": "std::string"
    },
    {
      "access": "protected",
      "comment": "Subscriber topic to listen for trigger msgs to leave the formation",
      "id": 1679753874536868957,
      "key": "leave_formation_topic",
      "type": "std::string"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2741715629576575326,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 1,
      "name": "",
      "plan": 2425328142973735249,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 3997532517592149463,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2425328142973735249,
  "inheritBlackboard": false,
  "libraryName": "libalica-turtlesim",
  "masterPlan": false,
  "name": "Simulation",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "Make the formation. Leave the state if a trigger to do so is received",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 428513459178575844,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": "leave_formation_topic",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        },
        {
          "abstractPlan": "MakeFormation.pml#1889749086610694100",
          "comment": "",
          "configuration": null,
          "id": 496472308860171093,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2405597980801916441,
      "inTransitions": [
        2899549957744653913
      ],
      "name": "MakeFormation",
      "outTransitions": [
        478458671918151236
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 822.7358972162353,
        "y": 198.3822276323798
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "Wait until a trigger to join the formation is received",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "WaitForTrigger.beh#1320667069122998665",
          "comment": "",
          "configuration": null,
          "id": 569238231169711866,
          "keyMapping": {
            "input": [
              {
                "childKey": "topic",
                "parentKey": "join_formation_topic",
                "value": null
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3064881354318777214,
      "inTransitions": [
        721746124359822922
      ],
      "name": "WaitForTrigger",
      "outTransitions": [
        2899549957744653913
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 568.630826143019,
        "y": -17.646812576786328
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TeleportToRandomPosition.pml#847199748749956244",
          "comment": "",
          "configuration": null,
          "id": 108848092215946015,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 2741715629576575326,
      "id": 3997532517592149463,
      "inTransitions": [
        478458671918151236
      ],
      "name": "Teleport2RandomPosition",
      "outTransitions": [
        721746124359822922
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 428,
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
      "id": 478458671918151236,
      "inState": 2405597980801916441,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3997532517592149463,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1059510119570948194,
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
      "id": 721746124359822922,
      "inState": 3997532517592149463,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3064881354318777214,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2511083801945401071,
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
      "id": 2899549957744653913,
      "inState": 3064881354318777214,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2405597980801916441,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 32783968180087158,
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
