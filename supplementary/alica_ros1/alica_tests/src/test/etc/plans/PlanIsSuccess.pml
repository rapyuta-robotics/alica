{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3124386063659432983,
      "key": "Normal2Dummy",
      "type": "bool"
    },
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3558181855747400341,
      "key": "Dummy2Normal",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1522377375150,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1522377375148,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1522377375149,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1522377375148,
  "inheritBlackboard": false,
  "masterPlan": true,
  "name": "PlanIsSuccess",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/SuccessNormal.beh#2951008684180289642",
          "comment": "",
          "configuration": null,
          "id": 3260483930806017445,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1522377375150,
      "id": 1522377375149,
      "inTransitions": [
        1522377945069
      ],
      "name": "Normal",
      "outTransitions": [
        1522377944058
      ],
      "parentPlan": 1522377375148,
      "positionWeb": {
        "x": 420.0892857142857,
        "y": 260.31919642857144
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/SuccessDummy.beh#3505111757300078074",
          "comment": "",
          "configuration": null,
          "id": 3078258125660157248,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1522377929290,
      "inTransitions": [
        1522377944058
      ],
      "name": "Dummy",
      "outTransitions": [
        1522377945069,
        3694363277253985460
      ],
      "parentPlan": 1522377375148,
      "positionWeb": {
        "x": 722.3652578437917,
        "y": 160.87615779372496
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4341767214611490181,
      "inTransitions": [
        3694363277253985460
      ],
      "name": "End",
      "outTransitions": [],
      "parentPlan": 1522377375148,
      "positionWeb": {
        "x": 865.0147473263322,
        "y": 41.051844408155404
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
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 1522377944058,
      "inState": 1522377375149,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "Normal2Dummy"
          }
        ],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1522377929290,
      "pointsWeb": [
        {
          "x": 586,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1522377944921,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#3592699233854318376",
      "id": 1522377945069,
      "inState": 1522377929290,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "Dummy2Normal"
          }
        ],
        "output": []
      },
      "name": "MISSING_NAME",
      "outState": 1522377375149,
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
        "id": 1522377946607,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#2872265442510628524",
      "id": 3694363277253985460,
      "inState": 1522377929290,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4341767214611490181,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2098555926520572428,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
