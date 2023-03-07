{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2588811185061120132,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1681561023694361721,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1470076529035237446,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1681561023694361721,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "ValueMappingConditionTestPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2588811185061120132,
      "id": 1470076529035237446,
      "inTransitions": [],
      "name": "ValueMappingConditionsTestState",
      "outTransitions": [
        2734074089394058504
      ],
      "parentPlan": 1681561023694361721,
      "positionWeb": {
        "x": 441.3033033033033,
        "y": 42.13413413413414
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4153272650033656757,
      "inTransitions": [
        2734074089394058504
      ],
      "name": "ValueMappingConditionsSuccessState",
      "outTransitions": [],
      "parentPlan": 1681561023694361721,
      "positionWeb": {
        "x": 686,
        "y": 200
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
      "condition": "ConditionRepository.cnd#300456113176014157",
      "id": 2734074089394058504,
      "inState": 1470076529035237446,
      "keyMapping": {
        "input": [
          {
            "childKey": "stringInput",
            "parentKey": null,
            "value": "test"
          },
          {
            "childKey": "uintInput",
            "parentKey": null,
            "value": 14
          },
          {
            "childKey": "intInput",
            "parentKey": null,
            "value": -14
          },
          {
            "childKey": "doubleInput",
            "parentKey": null,
            "value": -2.3
          },
          {
            "childKey": "boolInput",
            "parentKey": null,
            "value": 1
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 4153272650033656757,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1834624659938225633,
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
