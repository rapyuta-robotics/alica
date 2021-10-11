{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1782718581605808075,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "TracingTestEntry",
      "plan": 2798866311782929984,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 1817522181941290968,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2798866311782929984,
  "masterPlan": true,
  "name": "TracingTestMasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 2317048855866339918,
      "inTransitions": [
        4591920804716698102
      ],
      "name": "EndTest",
      "outTransitions": [],
      "parentPlan": 2798866311782929984,
      "positionWeb": {
        "x": 685.96875,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TracingDisabledPlan.pml#3148641312534759067",
          "comment": "",
          "configuration": null,
          "id": 1519106341307237321,
          "name": ""
        }
      ],
      "entryPoint": 1782718581605808075,
      "id": 1817522181941290968,
      "inTransitions": [],
      "name": "InitTest",
      "outTransitions": [
        4591920804716698102
      ],
      "parentPlan": 2798866311782929984,
      "positionWeb": {
        "x": 427.984375,
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
      "id": 4591920804716698102,
      "inState": 1817522181941290968,
      "name": "",
      "outState": 2317048855866339918,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 657847900438516106,
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