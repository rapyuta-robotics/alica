{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1402488881800,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "MISSING_NAME",
      "plan": 1402488870347,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1402488881799,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1402488870347,
  "masterPlan": false,
  "name": "GoalPlan",
  "preCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": true,
    "id": 1402489131988,
    "name": "PreCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [],
    "variables": []
  },
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "test",
    "enabled": false,
    "id": 1403773741874,
    "name": "NewRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [
      {
        "comment": "",
        "id": 1403773772633,
        "name": "MISSING_NAME",
        "quantifierType": "all",
        "scope": 1402489152217,
        "sorts": [
          "test"
        ]
      }
    ],
    "variables": [
      1403773747758
    ]
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1402488881800,
      "id": 1402488881799,
      "inTransitions": [
        1402489205153
      ],
      "name": "Shoot",
      "outTransitions": [
        1402489173167
      ],
      "parentPlan": 1402488870347,
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
      "id": 1402489152217,
      "inTransitions": [
        1402489173167
      ],
      "name": "Miss",
      "outTransitions": [
        1402489205153,
        1402489216617
      ],
      "parentPlan": 1402488870347,
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
      "id": 1402489192198,
      "inTransitions": [
        1402489216617
      ],
      "name": "Scored",
      "outTransitions": [],
      "parentPlan": 1402488870347,
      "positionWeb": {
        "x": 944,
        "y": 200
      },
      "postCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": false,
        "id": 1402489620773,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 1402489173167,
      "inState": 1402488881799,
      "name": "MISSING_NAME",
      "outState": 1402489152217,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402489174338,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1402489205153,
      "inState": 1402489152217,
      "name": "MISSING_NAME",
      "outState": 1402488881799,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402489206278,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1402489216617,
      "inState": 1402489152217,
      "name": "MISSING_NAME",
      "outState": 1402489192198,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1402489218027,
        "name": "MISSING_NAME",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.1,
  "variables": [
    {
      "comment": "",
      "id": 1403773747758,
      "name": "test",
      "variableType": "test"
    }
  ]
}