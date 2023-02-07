{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 3028476245590852924,
      "key": "WaitForTriggerState2BlackboardMapConstantTest",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2887699715416306214,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 4610306063152670816,
      "positionWeb": {
        "x": 186,
        "y": 287
      },
      "state": 924545140283507160,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 4610306063152670816,
  "inheritBlackboard": false,
  "libraryName": "",
  "masterPlan": false,
  "name": "BlackboardTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3712638990152664591,
      "inTransitions": [
        3130108995589923014
      ],
      "name": "BlackboardTestSuccess",
      "outTransitions": [],
      "parentPlan": 4610306063152670816,
      "positionWeb": {
        "x": 1219,
        "y": 363
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2887699715416306214,
      "id": 924545140283507160,
      "inTransitions": [],
      "name": "WaitForTriggerState",
      "outTransitions": [
        464426070784430186
      ],
      "parentPlan": 4610306063152670816,
      "positionWeb": {
        "x": 486,
        "y": 302
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "BlackboardMapConstantTestPlan.pml#2390819177564329533",
          "comment": "",
          "configuration": null,
          "id": 4251604882448327244,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 3364273219038396425,
      "inTransitions": [
        464426070784430186
      ],
      "name": "BlackboardMapConstantTest",
      "outTransitions": [
        3130108995589923014
      ],
      "parentPlan": 4610306063152670816,
      "positionWeb": {
        "x": 904,
        "y": 335
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
      "id": 464426070784430186,
      "inState": 924545140283507160,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "WaitForTriggerState2BlackboardMapConstantTest",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 3364273219038396425,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1007584014468848906,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1",
      "id": 3130108995589923014,
      "inState": 3364273219038396425,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3712638990152664591,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1204246573426302524,
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
