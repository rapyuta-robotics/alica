{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4488468250406966071,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 631515556091266493,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1171453089016322268,
      "successRequired": true,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 631515556091266493,
  "inheritBlackboard": false,
  "masterPlan": false,
  "name": "FailurePlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 4488468250406966071,
      "id": 1171453089016322268,
      "inTransitions": [],
      "name": "Init",
      "outTransitions": [
        1446293122737278544
      ],
      "parentPlan": 631515556091266493,
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
      "id": 3487518754011112127,
      "inTransitions": [
        1446293122737278544
      ],
      "name": "Fail",
      "outTransitions": [
        1023566846009251524
      ],
      "parentPlan": 631515556091266493,
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
      "id": 3748960977005112327,
      "inTransitions": [
        1023566846009251524
      ],
      "name": "Failed",
      "outTransitions": [],
      "parentPlan": 631515556091266493,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "postCondition": null,
      "success": false,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1770682125085719690",
      "id": 1023566846009251524,
      "inState": 3487518754011112127,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3748960977005112327,
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
        "id": 2038762164340314344,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "conditions/ConditionRepository.cnd#1291995818541962959",
      "id": 1446293122737278544,
      "inState": 1171453089016322268,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3487518754011112127,
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
        "id": 4351457352348187886,
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
