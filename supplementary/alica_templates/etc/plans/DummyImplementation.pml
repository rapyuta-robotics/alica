{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2593906152181871206,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 4126421719858579722,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 400856976447099508,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 5,
  "id": 4126421719858579722,
  "masterPlan": false,
  "name": "DummyImplementation",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1823707990536087965,
      "inTransitions": [
        293344705861516112
      ],
      "name": "DummySuccess",
      "outTransitions": [],
      "parentPlan": 4126421719858579722,
      "positionWeb": {
        "x": 686,
        "y": 200
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1069042249878174940,
      "inTransitions": [
        161866564931915050
      ],
      "name": "DummyFailure",
      "outTransitions": [],
      "parentPlan": 4126421719858579722,
      "positionWeb": {
        "x": 685.2240566037735,
        "y": 400
      },
      "postCondition": null,
      "success": false,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2593906152181871206,
      "id": 400856976447099508,
      "inTransitions": [],
      "name": "DummyState",
      "outTransitions": [
        293344705861516112,
        161866564931915050
      ],
      "parentPlan": 4126421719858579722,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 293344705861516112,
      "inState": 400856976447099508,
      "name": "",
      "outState": 1823707990536087965,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3469760593538210700,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 161866564931915050,
      "inState": 400856976447099508,
      "name": "",
      "outState": 1069042249878174940,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 2084505765197602547,
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