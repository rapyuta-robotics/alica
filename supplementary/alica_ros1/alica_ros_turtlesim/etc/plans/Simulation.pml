{
  "blackboard": [],
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
  "implementationName": "TracedPlan",
  "inheritBlackboard": false,
  "libraryName": "alica_standard_library",
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
                "parentKey": null,
                "value": "leave_formation"
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
        2899549957744653913,
        2467937620932239445
      ],
      "name": "MakeFormation",
      "outTransitions": [
        3130748750248347138
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 748.684223242413,
        "y": 210.7241732946835
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
                "parentKey": null,
                "value": "join_formation"
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
        3130748750248347138
      ],
      "name": "WaitForTrigger",
      "outTransitions": [
        2899549957744653913
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 745.6957024063598,
        "y": -4.680536256620286
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
      "inTransitions": [],
      "name": "Teleport2RandomPosition",
      "outTransitions": [
        2467937620932239445
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
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#1",
      "id": 2467937620932239445,
      "inState": 3997532517592149463,
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
        "id": 3077426683382044276,
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
      "id": 3130748750248347138,
      "inState": 2405597980801916441,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3064881354318777214,
      "pointsWeb": [
        {
          "x": 942.9246486000045,
          "y": 148.29910968526622
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3886186416925459075,
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
