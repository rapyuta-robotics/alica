{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4059536593953041663,
      "isDynamic": false,
      "maxCardinality": 1,
      "minCardinality": 1,
      "name": "",
      "plan": 691392966514374878,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 2832176823961443072,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 691392966514374878,
  "inheritBlackboard": false,
  "libraryName": "",
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
          "keyMapping": {
            "input": [],
            "output": []
          },
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
      "id": 3077460522716760463,
      "inTransitions": [
        3626189722064037094
      ],
      "name": "",
      "outTransitions": [],
      "parentPlan": 691392966514374878,
      "positionWeb": {
        "x": 838.9307805596466,
        "y": 198.6097201767305
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#4547372457936774346",
      "id": 3626189722064037094,
      "inState": 2832176823961443072,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 3077460522716760463,
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
