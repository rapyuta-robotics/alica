{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2741715629576575326,
      "maxCardinality": 2147483647,
      "minCardinality": 1,
      "name": "",
      "plan": 2425328142973735249,
      "positionWeb": {
        "x": 194,
        "y": 334
      },
      "state": 3997532517592149463,
      "successRequired": false,
      "task": "TaskRepository.tsk#3310236980587704776"
    }
  ],
  "frequency": 0,
  "id": 2425328142973735249,
  "masterPlan": true,
  "name": "Master",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/Go2RandomPosition.beh#4085572422059465423",
          "comment": "",
          "configuration": null,
          "id": 586073573470828748,
          "name": ""
        }
      ],
      "entryPoint": 2741715629576575326,
      "id": 3997532517592149463,
      "inTransitions": [
        635844345274619238
      ],
      "name": "Init",
      "outTransitions": [
        3486027875296378577
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 328,
        "y": 308
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "Move.pml#1889749086610694100",
          "comment": "",
          "configuration": null,
          "id": 496472308860171093,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2405597980801916441,
      "inTransitions": [
        3486027875296378577
      ],
      "name": "Move",
      "outTransitions": [
        635844345274619238
      ],
      "parentPlan": 2425328142973735249,
      "positionWeb": {
        "x": 578,
        "y": 355
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 635844345274619238,
      "inState": 2405597980801916441,
      "name": "",
      "outState": 3997532517592149463,
      "pointsWeb": [
        {
          "x": 479,
          "y": 396
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1136497454350831106,
        "name": "Move2Init",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 3486027875296378577,
      "inState": 3997532517592149463,
      "name": "",
      "outState": 2405597980801916441,
      "pointsWeb": [
        {
          "x": 488,
          "y": 329
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1597434482701133956,
        "name": "Init2Move",
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