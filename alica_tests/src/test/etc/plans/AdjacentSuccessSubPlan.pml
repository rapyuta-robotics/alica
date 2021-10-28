{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3470417373268048093,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 1682631238618360548,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 656998006978148289,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1682631238618360548,
  "masterPlan": false,
  "name": "AdjacentSuccessSubPlan",
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
          "id": 3080693630052187961,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1181007688948643441,
      "inTransitions": [
        3143778092687974738
      ],
      "name": "",
      "outTransitions": [
        1390829819585906015
      ],
      "parentPlan": 1682631238618360548,
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
      "id": 496520533178003845,
      "inTransitions": [
        1390829819585906015
      ],
      "name": "",
      "outTransitions": [],
      "parentPlan": 1682631238618360548,
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
      "confAbstractPlanWrappers": [],
      "entryPoint": 3470417373268048093,
      "id": 656998006978148289,
      "inTransitions": [],
      "name": "",
      "outTransitions": [
        3143778092687974738
      ],
      "parentPlan": 1682631238618360548,
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
      "id": 3143778092687974738,
      "inState": 656998006978148289,
      "name": "",
      "outState": 1181007688948643441,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3441061963559991094,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1390829819585906015,
      "inState": 1181007688948643441,
      "name": "",
      "outState": 496520533178003845,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3875618235052823378,
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