{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1019181667380114069,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2390819177564329533,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 602978301530919579,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2390819177564329533,
  "inheritBlackboard": false,
  "libraryName": "",
  "masterPlan": false,
  "name": "BlackboardMapConstantTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/IncreaseCountByX.beh#1084111613399827667",
          "comment": "",
          "configuration": null,
          "id": 1360305241920069641,
          "keyMapping": {
            "input": [
              {
                "childKey": "increaseBy",
                "parentKey": null,
                "value": 1
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1019181667380114069,
      "id": 602978301530919579,
      "inTransitions": [],
      "name": "InitState",
      "outTransitions": [
        1814492430994326017
      ],
      "parentPlan": 2390819177564329533,
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
      "id": 4000951790164702175,
      "inTransitions": [
        1658993097517406800
      ],
      "name": "BlackboardMapConstantTestSuccess",
      "outTransitions": [],
      "parentPlan": 2390819177564329533,
      "positionWeb": {
        "x": 944,
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
          "abstractPlan": "behaviours/IncreaseCountByX.beh#1084111613399827667",
          "comment": "",
          "configuration": null,
          "id": 2061186418395123093,
          "keyMapping": {
            "input": [
              {
                "childKey": "increaseBy",
                "parentKey": null,
                "value": 1
              }
            ],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2937672201731359801,
      "inTransitions": [
        1814492430994326017
      ],
      "name": "MapConstantTestState",
      "outTransitions": [
        1658993097517406800
      ],
      "parentPlan": 2390819177564329533,
      "positionWeb": {
        "x": 686.8,
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
      "condition": "conditions/ConditionRepository.cnd#2163654295690873706",
      "id": 1658993097517406800,
      "inState": 2937672201731359801,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": null,
            "value": 2
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4000951790164702175,
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
        "id": 3743250903768144590,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#2163654295690873706",
      "id": 1814492430994326017,
      "inState": 602978301530919579,
      "keyMapping": {
        "input": [
          {
            "childKey": "numberOfCalls",
            "parentKey": null,
            "value": 1
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 2937672201731359801,
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
        "id": 170416006263862082,
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
