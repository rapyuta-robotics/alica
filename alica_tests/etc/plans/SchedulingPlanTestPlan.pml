{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 3771375519284262837,
      "key": "EntryState2CheckState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3798767797946988684,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1655763442597171911,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 3649593845137984720,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1655763442597171911,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "SchedulingPlanTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "PlanA.pml#1629895837159",
          "comment": "",
          "configuration": null,
          "id": 3295206102330080955,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 677911864817236086,
      "inTransitions": [
        3188291632074488263
      ],
      "name": "CheckState",
      "outTransitions": [
        2482077043168548640
      ],
      "parentPlan": 1655763442597171911,
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
      "id": 2193242081569489645,
      "inTransitions": [
        2482077043168548640
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1655763442597171911,
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
      "entryPoint": 3798767797946988684,
      "id": 3649593845137984720,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        3188291632074488263
      ],
      "parentPlan": 1655763442597171911,
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
      "condition": "ConditionRepository.cnd#2",
      "id": 2482077043168548640,
      "inState": 677911864817236086,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 2193242081569489645,
      "pointsWeb": [
        {
          "x": 844,
          "y": 229
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3252829040265282981,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 3188291632074488263,
      "inState": 3649593845137984720,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "EntryState2CheckState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 677911864817236086,
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
        "id": 4285879219088263102,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": []
}
