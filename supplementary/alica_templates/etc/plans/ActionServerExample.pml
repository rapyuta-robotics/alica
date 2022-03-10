{
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 1647616282106629095,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2379894799421542548,
      "positionWeb": {
        "x": 200,
        "y": 212
      },
      "state": 4209576477302433246,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 5,
  "id": 2379894799421542548,
  "masterPlan": false,
  "name": "ActionServerExample",
  "preCondition": null,
  "relativeDirectory": "",
  "requiresParameters": false,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "DummyImplementation.pml#4126421719858579722",
          "comment": "",
          "configuration": null,
          "id": 1145345720307901712,
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 2119574391126023630,
      "inTransitions": [
        430744406068167347
      ],
      "name": "ExecuteGoal",
      "outTransitions": [
        1354699620997961969
      ],
      "parentPlan": 2379894799421542548,
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
      "entryPoint": 1647616282106629095,
      "id": 4209576477302433246,
      "inTransitions": [
        1354699620997961969
      ],
      "name": "WaitForGoal",
      "outTransitions": [
        430744406068167347
      ],
      "parentPlan": 2379894799421542548,
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
      "id": 430744406068167347,
      "inState": 4209576477302433246,
      "name": "",
      "outState": 2119574391126023630,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 1886820548377048134,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "id": 1354699620997961969,
      "inState": 2119574391126023630,
      "name": "",
      "outState": 4209576477302433246,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 587249152722263568,
        "name": "",
        "pluginName": "DefaultPlugin",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    }
  ],
  "utilityThreshold": 0.0,
  "variables": [],
  "inheritBlackboard" : false,
  "blackboard" : [
    {
      "key" : "result",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is the result of the executed action"
    },
    {
      "key" : "feedback",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is the feedback of the executed action"
    },
    {
      "key" : "goal",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is the goal of the executed action"
    },
    {
      "key" : "cancel",
      "type" : "std::any",
      "access" : "protected",
      "defaultValue" : null,
      "comment" : "This is a flag for cancelling the execution of the current goal"
    }
  ]
}