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
        "x": 448,
        "y": 349
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
      "entryPoint": null,
      "id": 3748960977005112327,
      "inTransitions": [
        1023566846009251524
      ],
      "name": "Failed",
      "outTransitions": [],
      "parentPlan": 631515556091266493,
      "positionWeb": {
        "x": 854.0424867408265,
        "y": 336.57841346877973
      },
      "postCondition": null,
      "success": false,
      "type": "TerminalState",
      "variableBindings": []
    },
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
        "x": 651,
        "y": 337
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
        "x": 753.4866261331831,
        "y": 223.84072270725608
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 1023566846009251524,
      "inState": 3487518754011112127,
      "name": "",
      "outState": 3748960977005112327,
      "pointsWeb": [],
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
      "id": 1446293122737278544,
      "inState": 1171453089016322268,
      "name": "",
      "outState": 3487518754011112127,
      "pointsWeb": [],
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
