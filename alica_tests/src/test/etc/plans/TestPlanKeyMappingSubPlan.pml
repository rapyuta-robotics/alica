{
  "blackboard": [
    {
      "access": "protected",
      "comment": "",
      "id": 4597788342183292286,
      "key": "childPlanKey",
      "type": "int64"
    }
  ],
  "comment": "",
  "entryPoints": [
    {
      "comment": "",
      "id": 3003611501631176059,
      "isDynamic": false,
      "maxCardinality": 2147483647,
      "minCardinality": 0,
      "name": "",
      "plan": 1282671484567034780,
      "positionWeb": {
        "x": -295,
        "y": 419
      },
      "state": 2084254100610656731,
      "successRequired": false,
      "task": "taskrepository.tsk#1225112227903"
    }
  ],
  "frequency": 0,
  "id": 1282671484567034780,
  "implementationName": "",
  "inheritBlackboard": false,
  "libraryName": "alica-tests",
  "masterPlan": false,
  "name": "TestPlanKeyMappingSubPlan",
  "preCondition": null,
  "relativeDirectory": "",
  "runtimeCondition": null,
  "states": [
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": null,
      "id": 174223825844944713,
      "inTransitions": [
        3645158883338889118
      ],
      "name": "SuccessState",
      "outTransitions": [],
      "parentPlan": 1282671484567034780,
      "positionWeb": {
        "x": 736.1061061061062,
        "y": 564.1771771771772
      },
      "postCondition": null,
      "success": true,
      "type": "TerminalState",
      "variableBindings": []
    },
    {
      "comment": "",
      "confAbstractPlanWrappers": [],
      "entryPoint": 3003611501631176059,
      "id": 2084254100610656731,
      "inTransitions": [],
      "name": "TestPlanKeyMappingSubPlanEntryState",
      "outTransitions": [
        3645158883338889118
      ],
      "parentPlan": 1282671484567034780,
      "positionWeb": {
        "x": 297,
        "y": 450
      },
      "type": "State",
      "variableBindings": []
    }
  ],
  "synchronisations": [],
  "transitions": [
    {
      "comment": "",
      "condition": "ConditionRepository.cnd#3151033742636780340",
      "id": 3645158883338889118,
      "inState": 2084254100610656731,
      "keyMapping": {
        "input": [
          {
            "childKey": "value",
            "parentKey": "childPlanKey",
            "value": null
          }
        ],
        "output": []
      },
      "name": "",
      "outState": 174223825844944713,
      "pointsWeb": [],
      "preCondition": {
        "comment": "",
        "conditionString": "",
        "enabled": true,
        "id": 4102300704943613646,
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
