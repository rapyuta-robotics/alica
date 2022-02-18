{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3277312192440194145,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1889749086610694100,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 2299237921449867536,
      "successRequired": false,
      "task": "TaskRepository.tsk#3759439551323513525"
    },
    {
      "comment": "",
      "id": 4346694000146342467,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 1889749086610694100,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 4158797811607100614,
      "successRequired": false,
      "task": "TaskRepository.tsk#826983480584534597"
    }
  ],
  "frequency": 0,
  "id": 1889749086610694100,
  "masterPlan": false,
  "name": "Move",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": {
    "comment": "",
    "conditionString": "",
    "enabled": false,
    "id": 1288817888979746811,
    "name": "CircleRuntimeCondition",
    "pluginName": "DefaultPlugin",
    "quantifiers": [
      {
        "comment": "",
        "id": 4131856376940334089,
        "name": "",
        "quantifierType": "all",
        "scope": 1889749086610694100,
        "sorts": [
          "x",
          "y"
        ]
      }
    ],
    "variables": []
  },
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/GoTo.beh#4054297592460872311",
          "comment": "",
          "configuration": null,
          "id": 3768390995110815611,
          "name": ""
        }
      ],
      "entryPoint": 3277312192440194145,
      "id": 2299237921449867536,
      "inTransitions": [],
      "name": "AlignCircle",
      "outTransitions": [],
      "parentPlan": 1889749086610694100,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "behaviours/GoTo.beh#4054297592460872311",
          "comment": "",
          "configuration": null,
          "id": 139589851715028435,
          "name": ""
        }
      ],
      "entryPoint": 4346694000146342467,
      "id": 4158797811607100614,
      "inTransitions": [],
      "name": "Move2Center",
      "outTransitions": [],
      "parentPlan": 1889749086610694100,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [],
  "utilityThreshold": 0.0,
  "variables": []
}