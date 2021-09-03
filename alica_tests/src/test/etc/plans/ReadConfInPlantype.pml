{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1588103719479,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "1588103719479",
      "plan": 1588061801734,
      "positionWeb": {
        "x": 200,
        "y": 412
      },
      "state": 1588103714226,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1588061801734,
  "masterPlan": false,
  "name": "ReadConfInPlantype",
  "preCondition": null,
  "relativeDirectory": "Configurations",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 1588103719479,
      "id": 1588103714226,
      "inTransitions": [],
      "name": "Default Name",
      "outTransitions": [
        1588246141555,
        1588246144840
      ],
      "parentPlan": 1588061801734,
      "positionWeb": {
        "x": 428,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 1588246134801,
      "inTransitions": [
        1588246141555
      ],
      "name": "ConfA",
      "outTransitions": [],
      "parentPlan": 1588061801734,
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
      "id": 1588246136647,
      "inTransitions": [
        1588246144840
      ],
      "name": "ConfB",
      "outTransitions": [],
      "parentPlan": 1588061801734,
      "positionWeb": {
        "x": 686,
        "y": 400
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "MISSING_COMMENT",
      "id": 1588246141555,
      "inState": 1588103714226,
      "name": "FromDefault NameTo Default Name",
      "outState": 1588246134801,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1588246141557,
        "name": "1588246141557",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "MISSING_COMMENT",
      "id": 1588246144840,
      "inState": 1588103714226,
      "name": "FromDefault NameTo Default Name",
      "outState": 1588246136647,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": null,
        "enabled": true,
        "id": 1588246144841,
        "name": "1588246144841",
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