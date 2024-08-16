{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 429816023887602514,
      "key": "SuccessOnCondState2WrapperBSuccessState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 4028079325935958114,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 2869465844414224272,
      "positionWeb": {
        "x": 350,
        "y": 377.99999237060547
      },
      "state": 3041287508452800918,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 2869465844414224272,
  "implementationName": "",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "SuccessOnCondWrapperBPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 27419054733292598,
      "inTransitions": [
        796950042414573400
      ],
      "name": "WrapperBSuccessState",
      "outTransitions": [],
      "parentPlan": 2869465844414224272,
      "positionWeb": {
        "x": 843.5743777534542,
        "y": 560.1825495149901
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "SuccessOnCondPlan.pml#3153116020668535682",
          "comment": "",
          "configuration": null,
          "id": 40364270334151145,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": 4028079325935958114,
      "id": 3041287508452800918,
      "inTransitions": [],
      "name": "SuccessOnCondState",
      "outTransitions": [
        796950042414573400
      ],
      "parentPlan": 2869465844414224272,
      "positionWeb": {
        "x": 546,
        "y": 363.99999237060547
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3592699233854318376",
      "id": 796950042414573400,
      "inState": 3041287508452800918,
      "keyMapping": {
        "input": [
          {
            "childKey": "result",
            "parentKey": "SuccessOnCondState2WrapperBSuccessState",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 27419054733292598,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 914907830776317719,
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
