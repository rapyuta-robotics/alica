{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4059536593953041663,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 691392966514374878,
      "positionWeb": {
        "x": 311,
        "y": 380
      },
      "state": 2832176823961443072,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 691392966514374878,
  "masterPlan": true,
  "name": "TestTracingMasterPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestTracingSubPlan.pml#1482512794732634139",
          "comment": "",
          "configuration": null,
          "id": 129367624571862066,
          "name": ""
        }
      ],
      "entryPoint": 4059536593953041663,
      "id": 2832176823961443072,
      "inTransitions": [],
      "name": "",
      "outTransitions": [
        3626189722064037094
      ],
      "parentPlan": 691392966514374878,
      "positionWeb": {
        "x": 725,
        "y": 377
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 3077460522716760463,
      "inTransitions": [
        3626189722064037094
      ],
      "name": "",
      "outTransitions": [],
      "parentPlan": 691392966514374878,
      "positionWeb": {
        "x": 1130,
        "y": 408
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "id": 3626189722064037094,
      "inState": 2832176823961443072,
      "name": "",
      "outState": 3077460522716760463,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1840401110297459509,
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