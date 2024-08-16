{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "defaultValue": null,
      "id": 1487312086401574568,
      "key": "EntryState2CheckState",
      "type": "bool"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 2547426852461060425,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "EntryState",
      "plan": 1546990961149934975,
      "positionWeb": {
        "x": 184,
        "y": 206.7125015258789
      },
      "state": 4549281436324771447,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1546990961149934975,
  "implementationName": "UntracedPlan",
  "inheritBlackboard": false,
  "isInterface": false,
  "libraryName": "alica_standard_library",
  "masterPlan": false,
  "name": "TestRepeatedRunsPlan",
  "preCondition": null,
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [
        {
          "abstractPlan": "TestRepeatedRunsBeh.beh#2555589216511791234",
          "comment": "",
          "configuration": null,
          "id": 1737723321300253952,
          "keyMapping": {
            "input": [],
            "output": []
          },
          "name": ""
        }
      ],
      "entryPoint": null,
      "id": 152409092979719560,
      "inTransitions": [
        1167813446747500893
      ],
      "name": "CheckState",
      "outTransitions": [
        2852738498720358625
      ],
      "parentPlan": 1546990961149934975,
      "positionWeb": {
        "x": 606,
        "y": 246.7125015258789
      },
      "type": "State",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 4516278134008548560,
      "inTransitions": [
        2852738498720358625
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1546990961149934975,
      "positionWeb": {
        "x": 842,
        "y": 302.7125015258789
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 2547426852461060425,
      "id": 4549281436324771447,
      "inTransitions": [],
      "name": "EntryState",
      "outTransitions": [
        1167813446747500893
      ],
      "parentPlan": 1546990961149934975,
      "positionWeb": {
        "x": 343,
        "y": 226.7125015258789
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
      "id": 1167813446747500893,
      "inState": 4549281436324771447,
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
      "outState": 152409092979719560,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3828984180796219377,
        "name": "",
        "quantifiers": [],
        "variables": []
      },
      "synchronisation": null
    },
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#2",
      "id": 2852738498720358625,
      "inState": 152409092979719560,
      "keyMapping": {
        "input": [],
        "output": []
      },
      "name": "",
      "outState": 4516278134008548560,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 3768381439388866693,
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
