{
  "blackboard": [],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1747181955620635504,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2039053377176713134,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 72643319450343579,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2039053377176713134,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "AdjacentSuccessTestPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SuccessOnCondPlan.pml#3153116020668535682",
          "comment": "",
          "configuration": null,
          "id": 704656399979191907,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 1747181955620635504,
      "id": 72643319450343579,
      "inTransitions": [
        4586149331725958934
      ],
      "name": "SuccessOnCondStateA",
      "outTransitions": [
        1124111964911548913
      ],
      "parentPlan": 2039053377176713134,
      "positionWeb": {
        "x": 428,
        "y": 200
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SuccessOnCondPlan.pml#3153116020668535682",
          "comment": "",
          "configuration": null,
          "id": 1427847311614550547,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 1813014648904251459,
      "inTransitions": [
        1124111964911548913
      ],
      "name": "SuccessOnCondStateB",
      "outTransitions": [
        4586149331725958934
      ],
      "parentPlan": 2039053377176713134,
      "positionWeb": {
        "x": 686,
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
      "id": 1124111964911548913,
      "inState": 72643319450343579,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 1813014648904251459,
      "pointsWeb": [
        {
          "x": 586,
          "y": 29
        }
      ],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3227220237533699258,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 4586149331725958934,
      "inState": 1813014648904251459,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 72643319450343579,
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
        "id": 3590963618136517342,
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
