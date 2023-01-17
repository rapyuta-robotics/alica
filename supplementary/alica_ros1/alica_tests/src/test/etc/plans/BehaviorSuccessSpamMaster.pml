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
  "name": "BehaviorSuccessSpamMaster",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/SuccessSpam.beh#1522377401286",
          "comment": "",
          "configuration": null,
          "id": 1587718663016,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718663016"
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
          "abstractPlan": "behaviours/SuccessSpam.beh#1522377401286",
          "comment": "",
          "configuration": null,
          "id": 1587718663020,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": "1587718663020"
        }
      ],
      "entryPoint": null,
      "id": 1522377929290,
      "inTransitions": [
        1522377944058
      ],
      "name": "Dummy",
      "outTransitions": [
        1522377945069
      ],
      "parentPlan": 1522377375148,
      "positionWeb": {
        "x": 716.6540178571429,
        "y": 133.74776785714286
      },
      "type": "State",
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
    }
  ],
  "utilityThreshold": 0.1,
  "variables": []
}
